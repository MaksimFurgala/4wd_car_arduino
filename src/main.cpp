#include <Arduino.h>
#include <SoftwareSerial.h>
#include <NewPing.h>

// Порты для сенсоров.
#define PIN_TRIG_FRONT A1
#define PIN_ECHO_FRONT A0
#define PIN_TRIG_LEFT A3
#define PIN_ECHO_LEFT A2
#define PIN_TRIG_RIGHT A5
#define PIN_ECHO_RIGHT A4

#define PIN_TX 10
#define PIN_RX 11

#define PIN_ENA 9
#define PIN_IN1 7
#define PIN_IN2 6

#define PIN_ENB 3
#define PIN_IN3 5
#define PIN_IN4 4

#define MAX_DISTANCE 400
#define STOP_DISTANCE 25 // было 20

// Сканеры (передний, левый, правый)
NewPing sonarFront(PIN_TRIG_FRONT, PIN_ECHO_FRONT, MAX_DISTANCE);
NewPing sonarLeft(PIN_TRIG_LEFT, PIN_ECHO_LEFT, MAX_DISTANCE);
NewPing sonarRight(PIN_TRIG_RIGHT, PIN_ECHO_RIGHT, MAX_DISTANCE);

SoftwareSerial mySerial(PIN_TX, PIN_RX); // модуль bluetooth HC-06

char currentChar = ' ';
bool autopilotMode = false;

void driveForward();
void turnRight();
void turnLeft();
void driveBack();
void stopRobot();
void setAutopilotMode(char mode);
long checkDistance(NewPing sonar);

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
  long distanceFront = checkDistance(sonarFront);
  long distanceLeft = checkDistance(sonarLeft);
  long distanceRight = checkDistance(sonarRight);

  Serial.print("Distance: "); // Выводим расстояние
  Serial.println(distanceFront);

  // if (distance < STOP_DISTANCE && distance != 0 && currentChar == 'F')
  // {
  //   stopRobot();
  // }

  // Если мы принимаем сигнал от bluetooth, то обрабатываем входные символы с порта.
  if (mySerial.available())
  {
    currentChar = mySerial.read();
    mySerial.println(currentChar);

    // if (distance < STOP_DISTANCE && distance != 0 && currentChar == 'F')
    //   return;

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
  Установка режима автопилота (вкл/выкл)
*/
void setAutopilotMode(char mode)
{
  if (mode == 'X')
  {
    autopilotMode = true;
  }
  else if (mode == 'x')
  {
    autopilotMode = false;
  }
}

/*
 * Проверить дистанцию до возможного препятствия.
 */
long checkDistance(NewPing sonar)
{
  delay(50);
  return sonar.ping_cm();
}