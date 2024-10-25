#define SOLENOID_PIN 25

void solenoidSetup() {
  pinMode(SOLENOID_PIN, OUTPUT);
}

void handleSolenoidInstruction(Instruction instr, int i) {
  if (instr.motors[i] == "solenoid") {
    solenoidPulse(instr.params[i]);
  }
}

void solenoidPulse(int ms) {
  digitalWrite(SOLENOID_PIN, 1);
  delay(ms);
  digitalWrite(SOLENOID_PIN, 0);
}