#include <Servo.h>

#define groovingAngle 48
#define fingerAngle 87
#define presserAngle 157
#define initialAngle 0

#define SERVO_PIN 25

char toolSelected = 'n';
// (n)o tool selected
// (g)rooving tool selected
// (f)inger tool selected
// (p)resser tool selected

Servo toolHolder;

bool servoRunning = false;
int angle = initialAngle;
int desiredAngle;
int angleIncrement;

int servoDelay = 50;
unsigned long prevTime;

void servoSetup() {
  toolHolder.write(0);
  toolHolder.attach(SERVO_PIN);
}

void beginServo(int desired) {
  servoRunning = 1;
  desiredAngle = desired;
  prevTime = millis();
  if (angle < desiredAngle) {
    angleIncrement = 1;
  } else if (angle > desiredAngle) {
    angleIncrement = -1;
  } else {
    angleIncrement = 0;
  }
}

void runServo() {
  if (servoRunning) {
    if (millis() > prevTime + servoDelay) {
      prevTime = millis();

      angle += angleIncrement;
      toolHolder.write(angle);
    }

    if (angle == desiredAngle) {
      servoRunning = false;
    }
  }
}

void handleServoInstruction(Instruction instr, int i) {
  if (instr.motors[i] == "holder") {
    int angleChosen = int(instr.params[i]);

    if (angleChosen == 0) beginServo(initialAngle);
    else if (angleChosen == 1) beginServo(presserAngle);
    else if (angleChosen == 2) beginServo(fingerAngle);
    else if (angleChosen == 3) beginServo(groovingAngle);
  } 
}