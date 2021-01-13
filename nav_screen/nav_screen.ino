#include <Automaton.h>
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
const int ACC_PIN = 2;             // Accessory pin, detect when car start
const int TILT_PIN = 9;            // Tilt button pin number
const int OPEN_PIN = 10;           // Open/close button pin number
const int SCREEN_PIN = 11;         // SCREEN PIN to turn off or on the screen
const int OS_PIN = 12;             // OS BUTTON
const int OS_POWER_RELAY_PIN = 4; // OS POWER RELAY
const int BUTTONDELAY = 400;       // Minimum time between button presses
const int ACC_DETECT_DELAY = 3000;   // Time (ms) that ACC needs to be on before car is considered 'on'
const int HOODOPENEDVALUE = 150;   // Analogue potentiometer value when hood is open
const int HOODCLOSEDVALUE = 850;   // Analogue potentiometer value when hood is closed
const int HOODPOSTOLERANCE = 10;   // Analogue potentiometer value tolerance
const int TILTVALUE = 5;          // Difference potentiometer value for each tilt
const int MAXTILT = 2;             // Max hood tilt levelEvtManager mgr;

// Defining Global Variables
int motorRunTime = 0;                   // Time the motor has run for
int previousHoodState = CLOSE;

// Automates
Atm_button openButton;
Atm_button tiltButton;
Atm_digital accPulse;
Atm_bit toggleHood;
Atm_bit osPower;
Atm_step tiltStep;
Atm_timer timerPowerScreen;

void setup() {
  pinMode(RPWM_PIN, OUTPUT);
  pinMode(LPWM_PIN, OUTPUT);
  pinMode(L_EN_PIN, OUTPUT);
  pinMode(R_EN_PIN, OUTPUT);
  pinMode(TILT_PIN, INPUT);
  pinMode(OPEN_PIN, INPUT);
  pinMode(SCREEN_PIN, OUTPUT);
  pinMode(ACC_PIN, INPUT);
  pinMode(OS_PIN, OUTPUT);
  pinMode(OS_POWER_RELAY_PIN, OUTPUT);

  digitalWrite(RPWM_PIN, LOW);
  digitalWrite(LPWM_PIN, LOW);
  digitalWrite(L_EN_PIN, LOW);
  digitalWrite(R_EN_PIN, LOW);
  digitalWrite(TILT_PIN, HIGH);
  digitalWrite(OPEN_PIN, HIGH);
  digitalWrite(SCREEN_PIN, LOW);
  digitalWrite(OS_PIN, HIGH);
  digitalWrite(OS_POWER_RELAY_PIN, LOW);

  Serial.begin(9600);

  setupEvents();
}

void setupEvents() {
  openButton.begin(OPEN_PIN)
  .longPress( 2, 400 )
  .onPress(1, openButtonChanged) 
  .onPress(2, togglePowerScreen);

  tiltButton.begin(TILT_PIN)
  .onPress(tiltButtonChanged);

  accPulse.begin(ACC_PIN, ACC_DETECT_DELAY)
  .onChange(LOW, accTurnOff)
  .onChange(HIGH, accTurnOn);

  osPower.begin()
  .onChange(true, startOS)
  .onChange(false, stopOS);

  toggleHood.begin()
  .onChange(false, closeHood)
  .onChange(true, openHood);

  tiltStep.begin();
  tiltStep.onStep(0, tiltHood);
  tiltStep.onStep(1, tiltHood);
  tiltStep.onStep(2, openHood);

  timerPowerScreen.begin(500)
  .onTimer(turnPinScreen);

  if(digitalRead(ACC_PIN) == LOW) {
    sleepNow();
  }
}

void accTurnOn(int idx, int v, int up ) {
  Serial.println("ACC ON");
  if(toggleHood.state()){
    toggleHood.trigger(toggleHood.EVT_REFRESH);
  }
}

void accTurnOff(int idx, int v, int up ) {
  Serial.println("ACC OFF");

  if (toggleHood.state()) {
    Serial.println("HOOD OPEN");
    // TOOD: Dont turn off screen, because the screen got a memory
    operateHood(CLOSE);
  }
  osPower.trigger(osPower.EVT_OFF);
  sleepNow();
}

void sleepNow()
{
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
}


void loop() {
  automaton.run();
}
