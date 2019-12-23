#include <avr/sleep.h>

const bool OPEN = true;
const bool CLOSE = false;
const bool HOODOPENED = true;
const bool HOODCLOSED = false;
const int POTENTIOMETER_PIN = A5; // Hood potentiometer
const int RPWM_PIN = 5;
const int LPWM_PIN = 6;
const int L_EN_PIN = 7;
const int R_EN_PIN = 8;
const int ACC_PIN = 2;           // Accessory pin, detect when car start
const int TILT_PIN = 9;          // Tilt button pin number
const int OPEN_PIN = 10;         // Open/close button pin number
const int LED_PIN = 11;          // LED BUTTON
const int OS_PIN = 12;           // OS BUTTON
const int BUTTONDELAY = 400;     // Minimum time between button presses
const int ACCDETECTDELAY = 2000; // Time (ms) that ACC needs to be on before car is considered 'on'
const int HOODOPENEDVALUE = 194; // Analogue potentiometer value when hood is open
const int HOODCLOSEDVALUE = 980; // Analogue potentiometer value when hood is closed
const int HOODPOSTOLERANCE = 10; // Analogue potentiometer value tolerance
const int TILTVALUE = 30;        // Difference potentiometer value for each tilt
const int MAXTILT = 2;           // Max hood tilt level

// Defining Global Variables
boolean osOff = false;                  // Initialise Operating System state
boolean carOff = true;                  // Initialise carOff state
boolean onHoodStatus = HOODCLOSED;      // Track desired hood status when car on, controlled by checkOpenButton()
boolean currentHoodStatus = HOODCLOSED; // Track current physical hood status, controlled by operateHood()
int motorRunTime = 0;                   // Time the motor has run for
int tiltLevel = 0;                      // Initialise hood tilt level (0:none - 2:tilted

void setup()
{

  pinMode(RPWM_PIN, OUTPUT);
  pinMode(LPWM_PIN, OUTPUT);
  pinMode(L_EN_PIN, OUTPUT);
  pinMode(R_EN_PIN, OUTPUT);
  pinMode(TILT_PIN, INPUT);
  pinMode(OPEN_PIN, INPUT);
  pinMode(LED_PIN, OUTPUT);
  pinMode(ACC_PIN, INPUT);
  pinMode(OS_PIN, OUTPUT);

  digitalWrite(RPWM_PIN, LOW);
  digitalWrite(LPWM_PIN, LOW);
  digitalWrite(L_EN_PIN, LOW);
  digitalWrite(R_EN_PIN, LOW);
  digitalWrite(TILT_PIN, HIGH);
  digitalWrite(OPEN_PIN, HIGH);
  digitalWrite(LED_PIN, HIGH);
  digitalWrite(OS_PIN, HIGH);

  Serial.begin(9600);
}

void loop()
{
  if (digitalRead(ACC_PIN) == HIGH)
  { // Accessories are on (car on)
    // Serial.println("ACC ON");
    checkResume();     // Check if the car was previously off
    checkOpenButton(); // Check if the Open button has been pressed
    checkTiltButton(); // Check if the Tilt button has been pressed
  }
  else
  { // Accessories are off (car off)
    Serial.println("ACC OFF");
    if (carOff)
    {
      sleepNow();
    }
    else
    {
      checkOff();
    }
  }
}

void checkResume()
{
  /*
   * Check if we are resuming from a 'carOff' event, restore previous hood position
   */
  if (carOff == true) // Only run if car off
  {
    delay(ACCDETECTDELAY);            // Wait a ACCDETECTDELAY time
    if (digitalRead(ACC_PIN) == HIGH) // Check to see if Accessories are still on
    {
      if (onHoodStatus == HOODOPENED)
      {
        operateHood(OPEN, false); // Restore the previous hood position
        restorePosition();
      }
      carOff = false;
    }
  }
}

void checkOff()
{
  /*
   * Check if the car is off and closes the hood
   */
  Serial.println("CHECK OFF");
  delay(ACCDETECTDELAY);           // Wait a ACCDETECTDELAY time
  if (digitalRead(ACC_PIN) == LOW) // Check if Accessories is still off
  {
    if (getPotentiometerValue() < (HOODCLOSEDVALUE - HOODPOSTOLERANCE)) // Hood is open, close it
    {
      operateHood(CLOSE, false);
    }
    carOff = true;
  }
}

void checkOpenButton()
{
  /*
   * Check if the Open button has been pressed and perform the open action
   */
  int openButtonState = digitalRead(OPEN_PIN); // Get the current state of the Open button
  if (openButtonState == LOW)                  // Button has been pressed
  {
    Serial.println("OPEN BUTTON");

    if (getPotentiometerValue() < (HOODCLOSEDVALUE - HOODPOSTOLERANCE)) // Hood is open
    {
      Serial.println("HOOD IS OPEN");
      if (osOff == false) // If OS is started we stop it
      {
        stopOS();
      }
      operateHood(CLOSE, false);
      onHoodStatus = HOODCLOSED;
    }
    else // Hood is closed
    {
      Serial.println("HOOD IS CLOSE");
      if (osOff) // If OS is stopped we started it
      {
        startOS();
      }
      operateHood(OPEN, false);
      restorePosition();
      onHoodStatus = HOODOPENED;
    }
    delay(BUTTONDELAY); // A delay so we have time to capture the button release
  }
}

void checkTiltButton()
{
  /*
   * Check if the Tilt button has been pressed and perform the tilt action
   */
  int tiltButtonState = digitalRead(TILT_PIN); // Get the current state of the Tilt button
  if (tiltButtonState == LOW)                  // Button has been pressed
  {
    Serial.println("TILT BUTTON");
    if (currentHoodStatus == HOODOPENED) // Hood is currently physically open
    {
      if (tiltLevel == MAXTILT) // If at max tilt, return to fully opened
      {
        operateHood(OPEN, false);
        tiltLevel = 0;
      }
      else // Otherwise, tilt it
      {
        operateHood(CLOSE, true);
        tiltLevel++;
      }
      delay(BUTTONDELAY); // A delay so we have time to capture the button release
    }
  }
}

void operateHood(bool dir, bool tilt)
{
  /*
   * Operate the navigation hood by driving the motor
   * 
   * Args:
   *    dir: A boolean determining the direction to drive the hood (OPEN or CLOSE)
   *    tilt: A boolean flag to signify if we only want to tilt the hood
   */
  if (tilt) // If we only want to tilt the hood, dir is ignored
  {
    String debugString = "Tilting hood, potentiometer: ";
    Serial.println(debugString + analogRead(POTENTIOMETER_PIN));
    startMotor(CLOSE);
    int currentValue = analogRead(POTENTIOMETER_PIN);
    Serial.println(analogRead(currentValue));

    // Run until we're fully open. If it's run for over 2s stop
    while (getPotentiometerValue() < currentValue + TILTVALUE)
    {
      Serial.println(analogRead(POTENTIOMETER_PIN));
      delay(100);
    }
    stopMotor();
  }
  else
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
      currentHoodStatus = HOODOPENED;
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
      currentHoodStatus = HOODCLOSED;
      Serial.println("Hood is closed");
    }
  }
}

void startMotor(bool dir)
{
  digitalWrite(L_EN_PIN, HIGH); // Enable the motor
  digitalWrite(R_EN_PIN, HIGH); // Enable the motor

  if (dir)
  {
    analogWrite(RPWM_PIN, 200);
  }
  else
  {
    analogWrite(LPWM_PIN, 200);
  }
}

void stopMotor()
{
  digitalWrite(L_EN_PIN, LOW);
  digitalWrite(R_EN_PIN, LOW); // Stop the motor
  analogWrite(LPWM_PIN, 0);
  analogWrite(RPWM_PIN, 0);
}

void restorePosition()
{
  /*
   * Restore the previous tilt position if any
   */
  for (int i = 0; i < tiltLevel; i++) // Tilt to previous desired level if any
  {
    delay(BUTTONDELAY);
    operateHood(CLOSE, true);
  }
}

int getPotentiometerValue()
{
  int sensorValue = analogRead(POTENTIOMETER_PIN);
  // Serial.println(sensorValue);
  return sensorValue;
}

void startOS()
{
  Serial.println("START OS");
  digitalWrite(OS_PIN, LOW);
  delay(1000);
  digitalWrite(OS_PIN, HIGH);
  osOff = false;
}

void stopOS()
{
  Serial.println("STOP OS");
  digitalWrite(OS_PIN, LOW);
  delay(5000);
  digitalWrite(OS_PIN, HIGH);
  osOff = true;
}

void sleepNow()
{
  /*
   * Setup an interrupt and enter sleep mode
   */
  Serial.println("Entering sleep mode");
  delay(100);
  set_sleep_mode(SLEEP_MODE_PWR_DOWN); // Set type of sleep mode
  sleep_enable();                      // Enable sleep mode
  attachInterrupt(0, wakeUp, HIGH);    // Use interrupt 0 (pin 2 ie. ACC input to wake device)
  sleep_mode();                        // Put device to sleep
  sleep_disable();                     // Execution resumes from here after waking up
  detachInterrupt(0);
  delay(100);
  Serial.println("Resuming from Sleep");
}

void wakeUp()
{
  Serial.println("WAKE UP");
  /*
   * The wakeUp() interrupt service routine will run when we get input from ACC (pin 2)
   * Since we just want the device to wake up we do nothing here
   */
}
