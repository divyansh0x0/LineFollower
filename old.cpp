#include<Arduino.h>
// -------- Motor Driver --------
#define AIN1 11
#define AIN2 12
#define PWMA 5

#define BIN1 9
#define BIN2 8
#define PWMB 6

#define STBY 10

// -------- Sensors --------
#define S0 A0
#define S1 A1
#define S2 A2
#define S3 A3
#define S4 A4

#define START_BUTTON 4

// -------- PID --------
float Kp = 105;
float Kd = 25;

int baseSpeed = 40;
int searchSpeed = 80;

int lastError = 0;

bool running = false;

void setup()
{
  pinMode(S0, INPUT);
  pinMode(S1, INPUT);
  pinMode(S2, INPUT);
  pinMode(S3, INPUT);
  pinMode(S4, INPUT);

  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);

  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);

  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, HIGH);

  pinMode(START_BUTTON, INPUT_PULLUP);
}

void loop()
{
  if (digitalRead(START_BUTTON) == LOW)
  {
    running = !running;
    delay(300);
  }

  if (!running) return;

  int s0 = !digitalRead(S0);
  int s1 = !digitalRead(S1);
  int s2 = !digitalRead(S2);
  int s3 = !digitalRead(S3);
  int s4 = !digitalRead(S4);

  int sensorSum = s0 + s1 + s2 + s3 + s4;

  // -------- LINE LOST --------
  if(sensorSum == 0)
  {
    motor(-searchSpeed, searchSpeed);   // spin left
    return;
  }

  // -------- PID --------
  int error =
      (-4 * s0) +
      (-2 * s1) +
      ( 0 * s2) +
      ( 2 * s3) +
      ( 4 * s4);

  int P = error;
  int D = error - lastError;

  int correction = Kp * P + Kd * D;

  lastError = error;

  int leftSpeed  = baseSpeed - correction;
  int rightSpeed = baseSpeed + correction;

  leftSpeed  = constrain(leftSpeed, -255, 255);
  rightSpeed = constrain(rightSpeed, -255, 255);

  motor(leftSpeed, rightSpeed);
}

// -------- MOTOR CONTROL --------
void motor(int left, int right)
{
  if(left >= 0){
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
    analogWrite(PWMA, left);
  }
  else{
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
    analogWrite(PWMA, -left);
  }

  if(right >= 0){
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);
    analogWrite(PWMB, right);
  }
  else{
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);
    analogWrite(PWMB, -right);
  }
}