
/* instruction:
  set of actions to take - started simultaneously, program only moves on when all actions complete
*/
typedef struct Instruction {
  String motors[6];
  float params[6]; 
} Instruction;

int instructionCount = 0;
const int nInstructions = 17;
Instruction instructions[nInstructions];
bool executingInstructions = true;
int returningStage = 0;

void setup() {
  Serial.begin(9600);
  motorSetup();
  solenoidSetup();
  servoSetup();

  /* GUIDE TO WRITING INSTRUCTIONS
    Each instruction has { {list of motors to control}, {list of corresponding params}}
    Params are:
      stepper motors - distance (in metres)
      solenoid - duration active (in ms)

    Limitations are:
      Can't have more thanr 2 stepper motors running in one instruction! (power issues)
      Can't have solenoid running with any other motor (timing issues - solenoid instructions are blocking)

    Make sure to change nInstructions to match the number of instructions
  */

  // PICK UP TOOL
  //instructions[0] = {{"vert1", "vert2"}, {0.055, 0.055}};
  //instructions[1] = {{"holder"}, {1}};
  // instructions[2] = {{"toolchange"}, {0.025}};
  //instructions[3] = {{"vert1", "vert2"}, {-0.055, -0.055}};
  //instructions[4] = {{"toolchange"}, {0.105}};
  instructions[0] = {{"vert1", "vert2"}, {0.05, 0.05}};

  // PRESS DOWN IN CENTRE
  instructions[1] = {{"toolchange"}, {0.243}};
  instructions[2] = {{"vert1", "vert2"}, {-0.04, -0.04}};
  instructions[3] = {{"vert1", "vert2"}, {0.05, 0.05}};

  // MOVE HANDS IN
  instructions[4] = {{"hands1", "hands2"}, {0.045, 0.045}};
  instructions[5] = {{"hands1", "hands2"}, {-0.045, -0.045}};

    // PRESS DOWN IN CENTRE
  instructions[6] = {{"vert1", "vert2"}, {-0.048, -0.048}};
  instructions[7] = {{"vert1", "vert2"}, {0.048, 0.048}};

  // MOVE HANDS IN
  instructions[8] = {{"hands1", "hands2"}, {0.046, 0.046}};
  instructions[9] = {{"hands1", "hands2"}, {-0.046, -0.046}};

    // PRESS DOWN IN CENTRE
  instructions[10] = {{"vert1", "vert2"}, {-0.048, -0.048}};
  instructions[11] = {{"vert1", "vert2"}, {0.048, 0.048}};


  // MOVE PRESSER AWAY
  instructions[12] = {{"toolchange"}, {-0.14}};

  // MOVE HANDS IN
  instructions[13] = {{"hands1", "hands2"}, {0.048, 0.048}};
  instructions[14] = {{"hands1", "hands2"}, {-0.048, -0.048}};

  // MOVE FINGER IN
  instructions[15] = {{"vert1", "vert2"}, {-0.001, -0.001}};
  instructions[16] = {{"finger"}, {0.195}};

}

void loop() {

  if (executingInstructions) runInstructions();
  if (returningStage != 0) returningStage = returnMotors(returningStage);

  runSteppers();
  runServo();
}

void runInstructions() {
  // if all motors have finished, move onto the next instruction
  if (!steppersRunning() && instructionCount < nInstructions) {
    Instruction instr = instructions[instructionCount];
    Serial.println("running instruction " + String(instructionCount));
    setSteppersEnabled(false);

    // handle each motor listed in the instruction
    for (int i = 0; i < sizeof(instr.motors) / sizeof(instr.motors[0]); i++) {
      handleStepperInstructions(instr, i);
      handleSolenoidInstruction(instr, i);
      handleServoInstruction(instr, i);
    }
    instructionCount++;
  }
}

