void openButtonChanged( int idx, int v, int up ) {
  if (v) {
    Serial.println("PRESSED OPEN BUTTON");
    toggleHood.trigger(toggleHood.EVT_TOGGLE);
  }
}

void tiltButtonChanged( int idx, int v, int up ) {
  if (v && toggleHood.state()) {
    tiltStep.trigger(tiltStep.EVT_STEP);
  }
}

void closeHood(int idx, int v, int up) {
  Serial.println("CLOSE HOOD");
  operateHood(CLOSE);
  togglePowerScreen();
}

void openHood(int idx, int v, int up) {
  Serial.println("OPEN HOOD");
  if(!osPower.state()){
    osPower.trigger(osPower.EVT_ON);
  }else{
    togglePowerScreen();
  }
  operateHood(OPEN);
}

void tiltHood( int idx, int v, int up ) {
  Serial.print( "Tilt HOOD " );
  tiltHood();
}

void tiltHood() {
  String debugString = "Tilting hood, potentiometer: ";
  Serial.println(debugString + analogRead(POTENTIOMETER_PIN));
  startMotor(CLOSE);
  int currentValue = analogRead(POTENTIOMETER_PIN);
  Serial.println(analogRead(currentValue));
  while (getPotentiometerValue() < currentValue + TILTVALUE)
  {
    Serial.println(analogRead(POTENTIOMETER_PIN));
    delay(100);
  }
  stopMotor();
}

void operateHood(bool dir)
{
  if (dir) // If direction is OPEN
  {
    String debugString = "Opening hood, potentiometer: ";
    Serial.println(debugString + analogRead(POTENTIOMETER_PIN));
    startMotor(OPEN);
    // Run until we're fully open. If it's run for over 2s stop
    while (getPotentiometerValue() > HOODOPENEDVALUE && motorRunTime < 200)
    {
      delay(100);
      motorRunTime++;
    }
    stopMotor();
    motorRunTime = 0;
    Serial.println("Hood is hopened");
  }
  else // If direction is CLOSE
  {
    String debugString = "Closing hood, potentiometer: ";
    Serial.println(debugString + analogRead(POTENTIOMETER_PIN));
    startMotor(CLOSE);
    // Run until we're fully closed. If it's run for over 2s stop;
    while (getPotentiometerValue() < HOODCLOSEDVALUE && motorRunTime < 200)
    {
      delay(100);
      motorRunTime++;
    }
    stopMotor();
    motorRunTime = 0;
    Serial.println("Hood is closed");
  }
}

bool isHoodHopen() {
  return getPotentiometerValue() < (HOODCLOSEDVALUE - HOODPOSTOLERANCE);
}

int getPotentiometerValue()
{
  int sensorValue = analogRead(POTENTIOMETER_PIN);
  // Serial.println(sensorValue);
  return sensorValue;
}
