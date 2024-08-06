#include <AccelStepper.h>
#include <EEPROM.h>
#include <Servo.h> // Librería para el servo motor

// Definir los pines para los motores y el servo
#define MOTOR_A_STEP_PIN 2
#define MOTOR_A_DIR_PIN 3
#define MOTOR_B_STEP_PIN 4
#define MOTOR_B_DIR_PIN 5
#define SERVO_PIN 9

// Crear instancias para los motores y el servo
AccelStepper motorA(AccelStepper::DRIVER, MOTOR_A_STEP_PIN, MOTOR_A_DIR_PIN);
AccelStepper motorB(AccelStepper::DRIVER, MOTOR_B_STEP_PIN, MOTOR_B_DIR_PIN);
Servo penServo; // Instancia del servo motor

// Variables globales
float stepMultiplier = 1.0;
long maxLength = 0;
float pageWidth = 200; // Definir ancho de la página
float pageHeight = 200; // Definir altura de la página
bool currentlyRunning = true;
bool usingAcceleration = true;
bool reportingPosition = false;
unsigned long lastOperationTime = 0; // Añadido para evitar errores de compilación

// Funciones auxiliares
long multiplier(int in) {
  return multiplier((long)in);
}

long multiplier(long in) {
  return in * stepMultiplier;
}

float multiplier(float in) {
  return in * stepMultiplier;
}

long divider(long in) {
  return in / stepMultiplier;
}

void changeLength(long tAl, long tBl) {
  float tA = float(tAl);
  float tB = float(tBl);
  lastOperationTime = millis();
  
  float currSpeedA = motorA.speed();
  float currSpeedB = motorB.speed();

  motorA.setSpeed(0.0);
  motorB.setSpeed(0.0);
  motorA.moveTo(tA);
  motorB.moveTo(tB);

  if (!usingAcceleration) {
    if (motorA.speed() < 0)
      currSpeedA = -currSpeedA;
    if (motorB.speed() < 0)
      currSpeedB = -currSpeedB;

    motorA.setSpeed(currSpeedA);
    motorB.setSpeed(currSpeedB);
  }

  while (motorA.distanceToGo() != 0 || motorB.distanceToGo() != 0) {
    if (currentlyRunning) {
      if (usingAcceleration) {
        motorA.run();
        motorB.run();
      } else {
        motorA.runSpeedToPosition();
        motorB.runSpeedToPosition();
      }
    }
  }
  
  reportPosition();
}

void changeLengthRelative(long tA, long tB) {
  lastOperationTime = millis();
  motorA.move(tA);
  motorB.move(tB);
  
  while (motorA.distanceToGo() != 0 || motorB.distanceToGo() != 0) {
    if (currentlyRunning) {
      if (usingAcceleration) {
        motorA.run();
        motorB.run();
      } else {
        motorA.runSpeedToPosition();
        motorB.runSpeedToPosition();
      }
    }
  }
  
  reportPosition();
}

long getMaxLength() {
  if (maxLength == 0) {
    maxLength = long(getMachineA(pageWidth, pageHeight) + 0.5);
    Serial.print(F("maxLength: "));
    Serial.println(maxLength);
  }
  return maxLength;
}

float getMachineA(float cX, float cY) {
  float a = sqrt(sq(cX) + sq(cY));
  return a;
}

float getMachineB(float cX, float cY) {
  float b = sqrt(sq((pageWidth) - cX) + sq(cY));
  return b;
}

void moveAxis(AccelStepper &m, int dist) {
  m.move(dist);
  while (m.distanceToGo() != 0) {
    if (currentlyRunning)
      m.run();
  }
  lastOperationTime = millis();
}

void reportPosition() {
  if (reportingPosition) {
    Serial.print("Position: ");
    Serial.print(divider(motorA.currentPosition()));
    Serial.print(", ");
    Serial.print(divider(motorB.currentPosition()));
    Serial.println();
  }
}

float getCartesianXFP(float aPos, float bPos) {
  float calcX = (sq((float)pageWidth) - sq((float)bPos) + sq((float)aPos)) / ((float)pageWidth * 2.0);
  return calcX;  
}

float getCartesianYFP(float cX, float aPos) {
  float calcY = sqrt(sq(aPos) - sq(cX));
  return calcY;
}

long getCartesianX(float aPos, float bPos) {
  long calcX = long((pow(pageWidth, 2) - pow(bPos, 2) + pow(aPos, 2)) / (pageWidth * 2));
  return calcX;  
}

long getCartesianX() {
  long calcX = getCartesianX(motorA.currentPosition(), motorB.currentPosition());
  return calcX;  
}

long getCartesianY() {
  return getCartesianY(getCartesianX(), motorA.currentPosition());
}

long getCartesianY(long cX, float aPos) {
  long calcY = long(sqrt(pow(aPos, 2) - pow(cX, 2)));
  return calcY;
}

void setup() {
  Serial.begin(9600);
  
  motorA.setMaxSpeed(1000); // Ajustar según sea necesario
  motorB.setMaxSpeed(1000); // Ajustar según sea necesario
  motorA.setAcceleration(100); // Ajustar según sea necesario
  motorB.setAcceleration(100); // Ajustar según sea necesario
  
  penServo.attach(SERVO_PIN);
  penServo.write(90); // Posición inicial del servo

  // Configuración inicial de motores y servo
}

void loop() {
  // Aquí va el código para mover los motores y el servo
  // Ejemplo: mover el motor A 1000 pasos y el motor B -500 pasos
  changeLength(1000, -500);

  // Ejemplo: mover el servo a 45 grados
  penServo.write(45);
  
  // Esperar 2 segundos
  delay(2000);

  // Repetir con otros movimientos si es necesario
}
