#include <SimpleKalmanFilter.h>
#include <NewPing.h>
#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

#pragma region khai báo chân cảm biến siêu âm
#define TRIG_C 10
#define ECHO_C 11
#define TRIG_L 12
#define ECHO_L 13
#define TRIG_R 8
#define ECHO_R 9
#pragma endregion

#pragma region khai báo chân động cơ
#define IN1 7
#define IN2 4
#define ENA 5
#define IN3 3
#define IN4 2
#define ENB 6
#pragma endregion

#pragma region khai báo biến toàn cục
// Khai báo khoảng cách đo
float minDist = 5.0;   // dưới 5 cm thì dừng xe (quá gần, nguy hiểm)
float maxDist = 100.0; // trên 100 cm thì dừng xe (mất vật, vượt tầm đo)
float setpoint = 30.0; // Khoảng cách mong muốn (cm)
float scaleFactor = 1.00;
float offset = 0;
float distanceC;
float distanceL;
float distanceR;
NewPing sensorC(TRIG_C, ECHO_C, maxDist);
NewPing sensorL(TRIG_L, ECHO_L, maxDist);
NewPing sensorR(TRIG_R, ECHO_R, maxDist);
SimpleKalmanFilter kf(2, 2, 0.3);
LiquidCrystal_I2C lcd(0x27, 16, 2);
#pragma endregion

#pragma region gain của e, de, v
float Ke = 1.0 / (100.0 - 24.80); // Gain scale error
float Kde = 0.01;                 // Gain scale derivative
float Kv = 90;                    // Gain scale fuzzy output -> PWM
float Kmin = 75;                  // Giá trị PWM tối thiểu
float percentKmin = 0.1;          // Phần trăm của Kv để lấy Kmin
#pragma endregion

// Khai báo biến lưu trữ giá trị lần trước
float prevError = 0.0;
float prevTime = 0.0;
unsigned long lastTime = 0;
float deltaErrorFiltered = 0.0;
float factorFilter = 0.1; // hằng số thời gian lọc đạo hàm

// Hàm scale gain e, de về [-1, 1], v thành [0,Kv]
float gain_Ke(float e)
{
  float x = e * Ke;
  if (x > 1)
    x = 1;
  if (x < -1)
    x = -1;
  return x;
}

float gain_Kde(float edot)
{
  float x = edot * Kde;
  if (x > 1)
    x = 1;
  if (x < -1)
    x = -1;
  return x;
}

float gain_Kv(float u_norm)
{
  float a = (Kv - Kmin) / (1 - percentKmin);
  float b = (Kmin - percentKmin * Kv) / (1 - percentKmin);
  if (u_norm < 0)
    b = -b;
  return u_norm * a + b;
}

// Hàm liên thuộc hình thang
float trapf(float x, float a, float b, float c, float d)
{
  if (x <= a || x >= d)
    return 0.0;
  if (x >= b && x <= c)
    return 1.0;
  if (x > a && x < b)
    return (x - a) / (b - a);
  if (x > c && x < d)
    return (d - x) / (d - c);
  return 0.0;
}

#pragma region Hàm liên thuộc
// Hàm liên thuộc của e
float eNB(float x) { return trapf(x, -2.0, -1.0, -0.6, -0.3); } // Negative Big
float eNS(float x) { return trapf(x, -0.6, -0.3, -0.3, 0.0); }  // Negative Small
float eZE(float x) { return trapf(x, -0.3, 0.0, 0.0, 0.3); }    // Zero
float ePS(float x) { return trapf(x, 0.0, 0.3, 0.3, 0.6); }     // Positive Small
float ePB(float x) { return trapf(x, 0.3, 0.6, 1.0, 2.0); }     // Positive Big

// Hàm liên thuộc của de
float deNB(float x) { return trapf(x, -2.0, -1.0, -0.5, -0.2); } // Negative Big
float deNS(float x) { return trapf(x, -0.5, -0.2, -0.2, 0.0); }  // Negative Small
float deZE(float x) { return trapf(x, -0.2, 0.0, 0.0, 0.2); }    // Zero
float dePS(float x) { return trapf(x, 0.0, 0.2, 0.2, 0.5); }     // Positive Small
float dePB(float x) { return trapf(x, 0.2, 0.5, 1.0, 2.0); }     // Positive

// Hàm liên thuộc của v
float vNB = -1.0; // Negative Big
float vNM = -0.7; // Negative Medium
float vNS = -0.5; // Negative Small
float vZE = 0.0;  // Zero
float vPS = 0.5;  // Positive Small
float vPM = 0.7;  // Positive Medium
float vPB = 1.0;  // Positive Big
#pragma endregion

float FuzzyTracking(float e, float de)
{
  // Fuzzification - Tính độ liên thuộc
  float E[5] = {eNB(e), eNS(e), eZE(e), ePS(e), ePB(e)};
  float DE[5] = {deNB(de), deNS(de), deZE(de), dePS(de), dePB(de)};

  float table[5][5] = {{vNB, vNB, vNM, vNS, vZE},
                       {vNB, vNM, vNS, vZE, vPS},
                       {vNM, vNS, vZE, vPS, vPM},
                       {vNS, vZE, vPS, vPM, vPB},
                       {vZE, vPS, vPM, vPB, vPB}};

  float num = 0.0, den = 0.0;

  // Suy diễn và giải mờ (Weighted Average)
  for (int i = 0; i < 5; i++)
    for (int j = 0; j < 5; j++)
    {
      // Toán tử AND = min
      float firing_strength = min(E[i], DE[j]);

      // Tích lũy
      num += firing_strength * table[i][j];
      den += firing_strength;
    }

  // Tính đầu ra
  if (den != 0.0)
    return num / den;
  else
    return 0.0;
}

// Hàm điều khiển động cơ
void motorControl(int rightMotorSpeed, int leftMotorSpeed)
{
  // Dead zone - Vùng chết để tránh rung
  if (abs(rightMotorSpeed) < Kmin || abs(leftMotorSpeed) < Kmin)
  {

    // Dừng xe
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
    analogWrite(ENA, 0);
    analogWrite(ENB, 0);
    return;
  }
  // Motor phải
  if (rightMotorSpeed < 0)
  {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  }
  else if (rightMotorSpeed > 0)
  {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
  }
  else
  {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
  }

  // Motor trái
  if (leftMotorSpeed < 0)
  {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
  }
  else if (leftMotorSpeed > 0)
  {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
  }
  else
  {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
  }

  analogWrite(ENA, abs(rightMotorSpeed));
  analogWrite(ENB, abs(leftMotorSpeed));
}

float readSonarDistance(int trigPin, int echoPin)
{
  // Clear TRIG
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);

  // Send ultrasonic pulse
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Read echo pulse
  long duration = pulseIn(echoPin, HIGH, 30000); // Timeout 30 ms

  if (duration == 0)
    return -1; // Sensor failed

  float distanceCm = (duration * 0.0343 / 2);
  return distanceCm;
}

void setup()
{
  Serial.begin(9600); // Cấu hình Serial Communication

  // Cấu hình các chân
  pinMode(TRIG_C, OUTPUT);
  pinMode(ECHO_C, INPUT);
  pinMode(TRIG_L, OUTPUT);
  pinMode(ECHO_L, INPUT);
  pinMode(TRIG_R, OUTPUT);
  pinMode(ECHO_R, INPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);

  motorControl(0, 0); // Dừng động cơ ban đầu

  lastTime = millis();

  Serial.print("Setpoint: ");
  Serial.print(setpoint);
  Serial.println(" cm");
  Serial.println();

  lcd.init();
  lcd.backlight();
}

void loop()
{
  float rawDistanceC = readSonarDistance(TRIG_C, ECHO_C);
  float rawDistanceL = readSonarDistance(TRIG_L, ECHO_L);
  float rawDCistanceR = readSonarDistance(TRIG_R, ECHO_R);
  if (rawDistanceC < 0)
  {
    lcd.setCursor(0, 1);        // Dòng 2
    lcd.print("             "); // Xóa dòng để tránh dính chữ cũ
    lcd.setCursor(0, 1);
    lcd.print("Khong doc duoc");
    motorControl(0, 0); // Dừng xe khi mất tín hiệu
    motorControl(0, 0); // Dừng xe khi mất tín hiệu
    delay(100);
    return;
  }
  distanceC = kf.updateEstimate(rawDistanceC);
  if (distanceC < minDist || distanceC > maxDist)
  {
    lcd.setCursor(0, 1);        // Dòng 2
    lcd.print("             "); // Xóa dòng để tránh dính chữ cũ
    lcd.setCursor(0, 1);
    lcd.print("Khong doc duoc");
    motorControl(0, 0); // Dừng xe khi mất tín hiệu
    delay(100);
    return;
  }

  // 3. Tính thời gian delta
  unsigned long currentTime = millis();
  float dt = (currentTime - lastTime) / 1000.0; // ms → s

  if (dt < 0.001)
    dt = 0.001; // tránh chia 0
  lastTime = currentTime;

  // Sai số
  float error = distanceC - setpoint;

  // Đạo hàm thô
  float deltaError = (error - prevError) / dt;

  // Lọc đạo hàm (giảm nhiễu)
  deltaErrorFiltered =
      (factorFilter / (factorFilter + dt)) * deltaErrorFiltered +
      (dt / (factorFilter + dt)) * deltaError;

  // 5. Gain scaling
  float e_scaled = gain_Ke(error);
  float de_scaled = gain_Kde(deltaError);

  // 6. Fuzzy Controller
  float u_norm = FuzzyTracking(e_scaled, de_scaled);

  // 7. Gain output scaling
  int motorSpeed = (int)gain_Kv(u_norm);

  // 8. Điều khiển động cơ
  // if (abs(motorSpeed) < Kmin && error < 1.6 && abs(prevError - error) < 0.3)
  // {
  //   motorControl(-Kmin, -Kmin);
  //   delay(700); // Dừng xe trong 700ms nếu quá gần
  //   motorControl(0, 0);
  // }
  // else if (abs(motorSpeed) < Kmin && error > -1.6 && abs(prevError - error) < 0.3)
  // {
  //   motorControl(-Kmin, -Kmin);
  //   delay(700); // Dừng xe trong 700ms nếu quá gần
  //   motorControl(0, 0);
  // }
  prevError = error;
  motorControl(motorSpeed, motorSpeed);

  Serial.println(motorSpeed);
  lcd.setCursor(0, 0);
  lcd.print("               ");
  lcd.setCursor(0, 0);
  lcd.print("V= ");
  lcd.setCursor(3, 0);
  lcd.print(motorSpeed);
  lcd.setCursor(0, 1);          // Dòng 2
  lcd.print("               "); // Xóa dòng để tránh dính chữ cũ
  lcd.setCursor(0, 1);
  lcd.print("D= ");
  lcd.setCursor(3, 1);
  lcd.print(distanceC);
  // Đợi trước khi đọc lần tiếp theo
  delay(100);
}