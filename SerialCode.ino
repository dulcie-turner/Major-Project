void serialEvent() {
  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read();
    if (inChar == 's') {
      Serial.println("stopping");
      executingInstructions = false;

      stopSteppers();
    } else if (inChar == 'r') {
      Serial.println("returning");
      executingInstructions = false;

      returningStage = returnMotors(0);
    }
  }
}