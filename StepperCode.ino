#include <AccelStepper.h>

#define ACCELERATION 300
#define MAX_SPEED 1500
#define FINGER_MAX_SPEED 2000
#define SCREW_LEAD 0.008 // screw travels 8mm per rotation
#define STEPS_PER_REV 800 // 200 steps, in 1/2 microstep mode

#define HANDS_MAX_POS 0.06
#define HANDS_MIN_POS 0

#define VERT_MAX_POS 0.065
#define VERT_MIN_POS 0
#define VERT_WHEEL_MIN_POS 0.075

#define TOOLCHANGE_MAX_POS 0.3
#define TOOLCHANGE_MIN_POS 0

#define FINGER_MAX_POS 0.3
#define FINGER_MIN_POS 0

AccelStepper hands1(AccelStepper::DRIVER, 54, 55);
AccelStepper hands2(AccelStepper::DRIVER, 60, 61);
AccelStepper vert1(AccelStepper::DRIVER, 46, 48);
AccelStepper vert2(AccelStepper::DRIVER, 26, 28);
AccelStepper toolchange(AccelStepper::DRIVER, 36, 34);
AccelStepper finger(AccelStepper::DRIVER, 17, 23);

float hands1Pos = HANDS_MIN_POS;
float hands2Pos = HANDS_MIN_POS;
float vert1Pos = VERT_MIN_POS;
float vert2Pos = VERT_MIN_POS;
float toolchangePos = TOOLCHANGE_MIN_POS;
float fingerPos = FINGER_MIN_POS;

// these must actually be zero upon boot!
// (i.e. motors must be returned to position before powering off)

void motorSetup() {

  hands1.setMaxSpeed(MAX_SPEED);
  hands2.setMaxSpeed(MAX_SPEED);
  vert1.setMaxSpeed(MAX_SPEED);
  vert2.setMaxSpeed(MAX_SPEED);
  toolchange.setMaxSpeed(FINGER_MAX_SPEED);
  finger.setMaxSpeed(FINGER_MAX_SPEED);

  hands1.setAcceleration(ACCELERATION);
  hands2.setAcceleration(ACCELERATION);
  vert1.setAcceleration(ACCELERATION);
  vert2.setAcceleration(ACCELERATION);
  toolchange.setAcceleration(ACCELERATION);
  finger.setAcceleration(ACCELERATION);

  hands1.setEnablePin(38);
  hands2.setEnablePin(56);
  vert1.setEnablePin(62);
  vert2.setEnablePin(24);
  toolchange.setEnablePin(30);
  finger.setEnablePin(16);

  // invert enable pin
  hands1.setPinsInverted(false, false, true);
  hands2.setPinsInverted(false, false, true);
  vert1.setPinsInverted(false, false, true);
  vert2.setPinsInverted(false, false, true);
  toolchange.setPinsInverted(false, false, true);
  finger.setPinsInverted(false, false, true);

  hands1.enableOutputs();
  hands2.enableOutputs();
  vert1.enableOutputs();
  vert2.enableOutputs();
  toolchange.enableOutputs();
  finger.enableOutputs();

}

long screwDistToStepperDist(float distance) {
  // convert distance along the lead screw to a desired stepper distance
  // (distance can be positive or negative)

  // rotations needed = distance to travel / distance travelled per rotation
  // distance needed = rotations needed * number of steps per rotation
  return (distance / SCREW_LEAD) * STEPS_PER_REV;
}

float stepperPosToScrewPos(int pos) {
  // convert stepper position to distance along the lead screw
  return (pos * SCREW_LEAD) / STEPS_PER_REV;
}

bool steppersRunning() {
  if (hands1.isRunning() || hands2.isRunning() || vert1.isRunning() || vert2.isRunning() || toolchange.isRunning() || finger.isRunning()) return true;
  return false;
}

void setSteppersEnabled(bool enable) {
  if (enable) {
    vert1.enableOutputs();
    vert2.enableOutputs();
    hands1.enableOutputs();
    hands2.enableOutputs();  
    toolchange.enableOutputs();
    finger.enableOutputs();
  } else {
    vert1.disableOutputs();
    vert2.disableOutputs();
    hands1.disableOutputs();
    hands2.disableOutputs();
    toolchange.disableOutputs();
    finger.disableOutputs();
  }
}

float adjustSpecifiedDist(float oldPos, float distance, float minPos, float maxPos) {
  // adjust the travel distance to avoid collisions
  if (oldPos + distance > maxPos) {
    Serial.println("Too far along the screw! Wanted to travel " + String(distance) + "m from a position of " + String(oldPos));
    Serial.println("Moving " + String(maxPos - oldPos) + "m instead, to get to " + maxPos);
    return maxPos - oldPos; // only travel far enough to get you to the max position
  } else if (oldPos + distance < minPos) {
    Serial.println("Not far enough along the screw! Wanted to travel " + String(distance) + "m from a position of " + String(oldPos));
    Serial.println("Moving " + String(minPos - oldPos) + "m instead, to get to " + minPos);
    return minPos - oldPos; // only travel far enough to get you to the min position

  } else {
    return distance; // travel the specified amount
  }
}

void handleStepperInstructions(Instruction instr, int i) {
  // set each stepper motor to go to its next position
  if (instr.motors[i] == "hands1") {
    float adjustedDist = adjustSpecifiedDist(hands1Pos, instr.params[i], HANDS_MIN_POS, HANDS_MAX_POS);
    hands1.enableOutputs();
    hands1.move(screwDistToStepperDist(adjustedDist));

  } else if (instr.motors[i] == "hands2") {
    float adjustedDist = adjustSpecifiedDist(hands2Pos, instr.params[i], HANDS_MIN_POS, HANDS_MAX_POS);
    hands2.enableOutputs();
    hands2.move(screwDistToStepperDist(adjustedDist));

  } else if (instr.motors[i] == "vert1") {
    float adjustedDist = adjustSpecifiedDist(vert1Pos, instr.params[i], VERT_MIN_POS, VERT_MAX_POS);
    vert1.enableOutputs();
    vert1.move(screwDistToStepperDist(adjustedDist));

  } else if (instr.motors[i] == "vert2") {
    float adjustedDist = adjustSpecifiedDist(vert2Pos, instr.params[i], VERT_MIN_POS, VERT_MAX_POS);
    vert2.enableOutputs();
    vert2.move(screwDistToStepperDist(adjustedDist));

  } else if (instr.motors[i] == "toolchange") {
    float adjustedDist = adjustSpecifiedDist(toolchangePos, instr.params[i], TOOLCHANGE_MIN_POS, TOOLCHANGE_MAX_POS);
    toolchange.enableOutputs();
    toolchange.move(screwDistToStepperDist(adjustedDist));    

  } else if (instr.motors[i] == "finger") {
    float adjustedDist = adjustSpecifiedDist(fingerPos, instr.params[i], FINGER_MIN_POS, FINGER_MAX_POS);
    finger.enableOutputs();
    finger.move(screwDistToStepperDist(adjustedDist));
  }
}

int returnMotors(int stage) {
  if (stage == 0) {
    stopSteppers();
    return 1;
  } else {

    if (stage == 1) {
      if (!steppersRunning()) {
        setSteppersEnabled(false);   

        // return the hands first
        hands1.enableOutputs();
        hands2.enableOutputs();
        hands1.moveTo(0);
        hands2.moveTo(0);
        Serial.println("returning horizontal");
        return 2;
      } else {
        return 1;
      }
    } else if (stage == 2) {
      if (!steppersRunning() && !servoRunning) {
        beginServo(initialAngle);
        Serial.println("returning servo");
        return 3;
      } else {
        return 2;
      }
    } else if (stage == 3) {
      if (!steppersRunning() && !servoRunning) {
        setSteppersEnabled(false);   

       // then return the finger motors
        Serial.println("returning fingers");
        toolchange.enableOutputs();
        finger.enableOutputs();
        toolchange.moveTo(0);
        finger.moveTo(0);
        
        return 4;
      } else {
        return 3;
      }
    } else if (stage == 4) {
            if (!steppersRunning() && !servoRunning) {
        setSteppersEnabled(false);   

        // then return the vertical motors
        Serial.println("returning vert");
        vert1.enableOutputs();
        vert2.enableOutputs();
        vert1.moveTo(0);
        vert2.moveTo(0);

        return 5;
      } else {
        return 4;
      }
    } else if (stage >= 5) {
      return 0;
    }
  } 
}

void stopSteppers() {
  hands1.stop();
  hands2.stop();
  vert1.stop();
  vert2.stop();
  toolchange.stop();
  finger.stop();

  // setSteppersEnabled(false);
}

void runSteppers() {
  // run all stepper motors (will only move if a position change is required)
  vert1.run();
  vert2.run();
  hands1.run();
  hands2.run();
  toolchange.run();
  finger.run();

  // update motor positions
  vert1Pos = stepperPosToScrewPos(vert1.currentPosition());
  vert2Pos = stepperPosToScrewPos(vert2.currentPosition());
  hands1Pos = stepperPosToScrewPos(hands1.currentPosition());
  hands2Pos = stepperPosToScrewPos(hands2.currentPosition());
  toolchangePos = stepperPosToScrewPos(toolchange.currentPosition());
  fingerPos = stepperPosToScrewPos(finger.currentPosition());

  if (!vert1.isRunning()) vert1.disableOutputs();
  if (!vert2.isRunning()) vert2.disableOutputs();
  if (!hands1.isRunning()) hands1.disableOutputs();
  if (!hands2.isRunning()) hands2.disableOutputs();
  if (!toolchange.isRunning()) toolchange.disableOutputs();
  if (!finger.isRunning()) finger.disableOutputs();
}