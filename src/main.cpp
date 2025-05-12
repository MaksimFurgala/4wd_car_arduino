#include <Arduino.h>
#include <SoftwareSerial.h>
#include <NewPing.h>

#define PIN_TRIG 8
#define PIN_ECHO 12

#define PIN_TX 10
#define PIN_RX 11

#define PIN_ENA 9
#define PIN_IN1 7
#define PIN_IN2 6

#define PIN_ENB 3
#define PIN_IN3 5
#define PIN_IN4 4

#define MAX_DISTANCE 200
#define STOP_DISTANCE 20

NewPing sonar(PIN_TRIG, PIN_ECHO, MAX_DISTANCE); // ультразвуковой датчик HC-SR04
SoftwareSerial mySerial(PIN_TX, PIN_RX);         // модуль bluetooth HC-06

char currentChar = ' ';

void driveForward();
void turnRight();
void turnLeft();
void driveBack();
void stopRobot();
long checkDistance();

void setup()
{
  pinMode(10, INPUT);
  pinMode(11, OUTPUT);
  pinMode(PIN_ENA, OUTPUT);
  pinMode(PIN_IN1, OUTPUT);
  pinMode(PIN_IN2, OUTPUT);
  mySerial.begin(9600);
  Serial.begin(9600); // Включаем Serial для вывода в монитор
}

void loop()
{
  long distance = checkDistance();
  Serial.print("Distance: "); // Выводим расстояние
  Serial.println(distance);

  if (distance < STOP_DISTANCE && distance != 0 && currentChar == 'F')
  {
    stopRobot();
  }

  if (mySerial.available())
  {
    currentChar = mySerial.read();
    mySerial.println(currentChar);

    if (distance < STOP_DISTANCE && distance != 0 && currentChar == 'F')
      return;

    switch (currentChar)
    {
    case 'F':
      driveForward();
      break;
    case 'L':
      turnLeft();
      break;
    case 'R':
      turnRight();
      break;
    case 'B':
      driveBack();
      break;
    default:
      stopRobot();
      break;
    }
  }
}

/*
 * Движение вперед.
 */
void driveForward()
{
  digitalWrite(PIN_IN1, LOW);
  digitalWrite(PIN_IN2, HIGH);
  digitalWrite(PIN_IN3, LOW);
  digitalWrite(PIN_IN4, HIGH);
  analogWrite(PIN_ENA, 255);
  analogWrite(PIN_ENB, 255);
}

/*
 * Движение назад.
 */
void driveBack()
{
  digitalWrite(PIN_IN1, HIGH);
  digitalWrite(PIN_IN2, LOW);
  digitalWrite(PIN_IN3, HIGH);
  digitalWrite(PIN_IN4, LOW);
  analogWrite(PIN_ENA, 255);
  analogWrite(PIN_ENB, 255);
}

/**
 * Поворот вправо.
 */
void turnRight()
{
  digitalWrite(PIN_IN1, LOW);
  digitalWrite(PIN_IN2, HIGH);
  digitalWrite(PIN_IN3, HIGH);
  digitalWrite(PIN_IN4, LOW);
  analogWrite(PIN_ENA, 255);
  analogWrite(PIN_ENB, 255);
}

/**
 * Поворот влево.
 */
void turnLeft()
{
  digitalWrite(PIN_IN1, HIGH);
  digitalWrite(PIN_IN2, LOW);
  digitalWrite(PIN_IN3, LOW);
  digitalWrite(PIN_IN4, HIGH);
  analogWrite(PIN_ENA, 255);
  analogWrite(PIN_ENB, 255);
}

/*
 * Остановка.
 */
void stopRobot()
{
  analogWrite(PIN_ENA, 0);
  analogWrite(PIN_ENB, 0);
}

/*
 * Проверить дистанцию до возможного препятствия.
 */
long checkDistance()
{
  delay(50);
  return sonar.ping_cm();
}