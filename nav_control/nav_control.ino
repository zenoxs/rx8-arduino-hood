#include <HID-Project.h>
#include <HID-Settings.h>
#include <SoftwareSerial.h>

//Configuration
#define ENABLE_DEBUG_OUTPUT

//Depended parameters
#define BIT_LOW_LEVEL_DURATION_MIN  (1400)  //value in us
#define BIT_LOW_LEVEL_DURATION_MAX  (2000)  //value in us


#ifdef ENABLE_DEBUG_OUTPUT
#define DEBUG_PRINT(...)  if(Serial){ Serial.print(__VA_ARGS__); }
#else
#define DEBUG_PRINT(...)
#endif


//Constants
#define UNIT_AUDIO_PIN_INPUT_MODE   (INPUT_PULLUP)
#define GPS_PIN                     8U // only 8, 9, 10, 11, 14 (MISO), 15 (SCK), 16 (MOSI).
#define UNIT_AUDIO_PIN              2U

#define RX_TIMEOUT_MS               12U
#define IN_BUFFER_SIZE              96U

#define NIBBLE_RESET_BIT_POS        0x08

/* message mustbe aligned to bytes*/
typedef struct rxMessage {
  uint8_t target;
  uint8_t command;
  uint8_t data[];
} rxMessage_t;

typedef enum rxMessageTarget_e {
  Target_TapeDesk = 0x00,
  Target_Unknown = 0x01,
  Target_CDDesk = 0x03,
  Target_CDChangerExt = 0x05,
  Target_CDChangerUpper = 0x06,
  Target_MDDesk = 0x07,
  Target_BaseUnit = 0x08
} rxMessageTarget_t;

typedef enum rxMessageCommand_e {
  Command_Control = 0x01,
  Command_AnyBodyHome = 0x08,
  Command_WakeUp = 0x09
} rxMessageCommand_t;

typedef enum rxMessageSubCommand_e {
  SubCommand_Playback = 0x01,
  SubCommand_SeekTrack = 0x03,
  SubCommand_SetConfig = 0x04
} rxMessageSubCommand_t;

typedef enum SubConmmandPlayback_e {
  Playback_Play = 0x01,
  Playback_FF = 0x04,
  Playback_REW = 0x08,
  Playback_Stop = 0x60
} SubConmmandPlayback_t;

typedef enum SubCommandSetConfig_e {
  SetConfig_RepeatMode = 0x01,
  SetConfig_RandomMode = 0x02,
  SetConfig_FastForwarding = 0x10,
  SetConfig_FastRewinding = 0x20
} SubCommandSetConfig_t;

// data present in nibbles, byte equal nibble
//Wakeup notification
const uint8_t TAPECMD_POWER_ON[] =        {0x08, 0x08, 0x01, 0x02};         //Wake up notification
//Status messages: {target, command(status), arg1, arg2, checksum}
const uint8_t TAPECMD_STOPPED[] =         {0x08, 0x09, 0x00, 0x0C, 0x0E};   //0 - Stopped, C - not use desk
const uint8_t TAPECMD_PLAYING[] =         {0x08, 0x09, 0x04, 0x01, 0x05};   //4 - Playing, 1 - tape in use
const uint8_t TAPECMD_SEEKING[] =         {0x08, 0x09, 0x05, 0x01, 0x06};   //5 - seeking, 1 - tape in use
//Detailed status  {target, command(det. status), arg1, arg2, arg3, arg4, arg5, arg6, checksum}
const uint8_t TAPECMD_CASSETE_PRESENT[] = {0x08, 0x0B, 0x09, 0x00, 0x04, 0x00, 0x00, 0x0C, 0x03};
const uint8_t TAPECMD_PLAYBACK[] =        {0x08, 0x0B, 0x09, 0x00, 0x04, 0x00, 0x00, 0x01, 0x00};
const uint8_t TAPECMD_RANDOM_PLAY[] =     {0x08, 0x0B, 0x09, 0x00, 0x06, 0x00, 0x00, 0x01, 0x0E};
const uint8_t TAPECMD_REPEAT_PLAY[] =     {0x08, 0x0B, 0x09, 0x00, 0x05, 0x00, 0x00, 0x01, 0x0F};
const uint8_t TAPECMD_FAST_REWIND[] =     {0x08, 0x0B, 0x09, 0x03, 0x04, 0x00, 0x01, 0x01, 0x0E};
const uint8_t TAPECMD_FAST_FORWARD[] =    {0x08, 0x0B, 0x09, 0x02, 0x04, 0x00, 0x01, 0x01, 0x0D};

// audio
static uint8_t inNibblesBuffer[IN_BUFFER_SIZE] = {0U};
static uint8_t nibblesReceived = 0;
static uint8_t biteShiftMask = NIBBLE_RESET_BIT_POS;
static uint32_t rx_time_us = 0;
static uint32_t rx_time_ms = 0;

// gps
static char gpsCommandBuffer[15];
static int gpsCommandCursor = 0;
SoftwareSerial navControlSerial(GPS_PIN, GPS_PIN); // RX, TX

void setup() {
  navControlSerial.begin(2400);

  pinMode(UNIT_AUDIO_PIN, UNIT_AUDIO_PIN_INPUT_MODE);
  attachInterrupt(digitalPinToInterrupt(UNIT_AUDIO_PIN), collectInputData, CHANGE);

#ifdef ENABLE_DEBUG_OUTPUT
  Serial.begin(9600);
#endif
  Keyboard.begin();

  DEBUG_PRINT("Init....\r\n");
}

void loop() {
  if(navControlSerial.available() > 0) {
    navControlSerialEvent();
  }

  if ( ( millis() - rx_time_ms ) > RX_TIMEOUT_MS) {
    if (nibblesReceived != 0U ) {

      noInterrupts(); {
        DEBUG_PRINT("RX AUDIO[");
        DEBUG_PRINT(nibblesReceived);
        DEBUG_PRINT("] ");

        for (int i = 0; i < nibblesReceived; i++) {
          DEBUG_PRINT(inNibblesBuffer[i], HEX);
        }
        DEBUG_PRINT("\r\n");

        process_radio_message((rxMessage_t*)inNibblesBuffer);

        bufferReset();

        rx_time_ms = millis();

      } interrupts();
    }
  }
}

void navControlSerialEvent() {
  char inChar = (char)navControlSerial.read();
  if (inChar == 0) {
    gpsCommandCursor = 0;
  }
  gpsCommandBuffer[gpsCommandCursor] = inChar;
  gpsCommandCursor++;
  if (gpsCommandCursor > 12) {
    gpsCommandCursor = 0;
    process_gps_control();
  }
}

void process_gps_control() {
  DEBUG_PRINT("RX GPS[");
  DEBUG_PRINT(gpsCommandCursor);
  DEBUG_PRINT("] ");

  for (int i = 0; i < 12; i++) {
    DEBUG_PRINT(((uint8_t*)gpsCommandBuffer)[i], HEX);
  }

  DEBUG_PRINT("\r\n");

  //Right Arrow
  if (gpsCommandBuffer[6] == 0x4F and gpsCommandBuffer[7] == 0x4B and gpsCommandBuffer[8] == 0xFFFFFFCB and gpsCommandBuffer[9] == 0x7B and gpsCommandBuffer[10] == 0xFFFFFFCF ) {
    DEBUG_PRINT("Right");
    Keyboard.write(KEY_RIGHT_ARROW);
  }
  //Left Arrow
  if (gpsCommandBuffer[6] == 0xFFFFFFCF and gpsCommandBuffer[7] == 0x4B and gpsCommandBuffer[8] == 0x4B and gpsCommandBuffer[9] == 0xFFFFFFCF and gpsCommandBuffer[10] == 0xFFFFFFCF ) {
    DEBUG_PRINT("Left");
    Keyboard.write(KEY_LEFT_ARROW);
  }
  //Up Arrow
  if (gpsCommandBuffer[6] == 0x4F and gpsCommandBuffer[7] == 0x4B and gpsCommandBuffer[8] == 0xFFFFFFCB and gpsCommandBuffer[9] == 0xFFFFFFCF and gpsCommandBuffer[10] == 0xFFFFFFCF ) {
    DEBUG_PRINT("Up");
    Keyboard.write(KEY_UP_ARROW);
  }
  //Down Arrow
  if (gpsCommandBuffer[6] == 0xFFFFFFCF and gpsCommandBuffer[7] == 0x4F and gpsCommandBuffer[8] == 0x4B and gpsCommandBuffer[9] == 0xFFFFFFCB and gpsCommandBuffer[10] == 0xFFFFFFCF ) {
    DEBUG_PRINT("Down");
    Keyboard.write(KEY_DOWN_ARROW);
  }
  // Enter Button
  if (gpsCommandBuffer[6] == 0x4F and gpsCommandBuffer[7] == 0x7B and gpsCommandBuffer[8] == 0x4B and gpsCommandBuffer[9] == 0xFFFFFFCF and gpsCommandBuffer[10] == 0x7B ) {
    DEBUG_PRINT("Enter");
    Keyboard.press(KEY_RETURN);
    Keyboard.press(KEY_F1);
    Keyboard.releaseAll();
  }
  // RET Button
  if (gpsCommandBuffer[6] == 0xFFFFFFCF and gpsCommandBuffer[7] == 0xFFFFFFCB and gpsCommandBuffer[8] == 0x4B and gpsCommandBuffer[9] == 0x7B and gpsCommandBuffer[10] == 0x7B ) {
    DEBUG_PRINT("Ret");
    Keyboard.write(KEY_ESC);
  }
  // Zoom In Button (arrow to front) need to modify navit keys.h
  if (gpsCommandBuffer[6] == 0x4F and gpsCommandBuffer[7] == 0x7B and gpsCommandBuffer[8] == 0x4B and gpsCommandBuffer[9] == 0x7B and gpsCommandBuffer[10] == 0xFFFFFFCF ) {
    DEBUG_PRINT("Zoom in");
    Keyboard.write(KEY_PAGE_UP);
  }
  // Zoom out Button (arrow to back) need to modify navit keys.h
  if (gpsCommandBuffer[6] == 0xFFFFFFCF and gpsCommandBuffer[7] == 0x4F and gpsCommandBuffer[8] == 0x4B and gpsCommandBuffer[9] == 0xFFFFFFCB and gpsCommandBuffer[10] == 0x7B ) {
    DEBUG_PRINT("Zoom out");
    Keyboard.write(KEY_PAGE_DOWN);
  }
  // Menu
  if (gpsCommandBuffer[6] == 0x4F and gpsCommandBuffer[7] == 0xFFFFFFCF and gpsCommandBuffer[8] == 0x4B and gpsCommandBuffer[9] == 0xFFFFFFCF and gpsCommandBuffer[10] == 0xFFFFFFCB ) {
    DEBUG_PRINT("Menu");
    Keyboard.write(KEY_HOME);
  }
  // Voice Button
  if (gpsCommandBuffer[6] == 0x4F and gpsCommandBuffer[7] == 0xFFFFFFCF and gpsCommandBuffer[8] == 0x4B and gpsCommandBuffer[9] == 0x4F and gpsCommandBuffer[10] == 0x7B ) {
    DEBUG_PRINT("Voice");
    Keyboard.write('v');
  }
  // POS Button
  if (gpsCommandBuffer[6] == 0xFFFFFFCF and gpsCommandBuffer[7] == 0x7B and gpsCommandBuffer[8] == 0x4B and gpsCommandBuffer[9] == 0x7B and gpsCommandBuffer[10] == 0x7B ) {
    DEBUG_PRINT("Pos");
    Keyboard.write('p');
  }
  // Top Left
  if (gpsCommandBuffer[6] == 0x4F and gpsCommandBuffer[7] == 0x4F and gpsCommandBuffer[8] == 0x4B and gpsCommandBuffer[9] == 0x4F and gpsCommandBuffer[10] == 0xFFFFFFCF ) {
    DEBUG_PRINT("Top Left");
    Keyboard.press(KEY_UP_ARROW);
    Keyboard.press(KEY_LEFT_ARROW);
    Keyboard.releaseAll();
  }
  // Bottom Right
  if (gpsCommandBuffer[6] == 0x4F and gpsCommandBuffer[7] == 0x4F and gpsCommandBuffer[8] == 0x4B and gpsCommandBuffer[9] == 0xFFFFFFCB and gpsCommandBuffer[10] == 0xFFFFFFCF ) {
    DEBUG_PRINT("Bottom Right");
    Keyboard.press(KEY_DOWN_ARROW);
    Keyboard.press(KEY_RIGHT_ARROW);
    Keyboard.releaseAll();
  }
  // Top Right
  if (gpsCommandBuffer[6] == 0xFFFFFFCF and gpsCommandBuffer[7] == 0x7B and gpsCommandBuffer[8] == 0x4B and gpsCommandBuffer[9] == 0x7B and gpsCommandBuffer[10] == 0xFFFFFFCF ) {
    DEBUG_PRINT("Top Right");
    Keyboard.press(KEY_UP_ARROW);
    Keyboard.press(KEY_RIGHT_ARROW);
    Keyboard.releaseAll();
  }
  // Bottom Left
  if (gpsCommandBuffer[6] == 0xFFFFFFCF and gpsCommandBuffer[7] == 0xFFFFFFCF and gpsCommandBuffer[8] == 0x4B and gpsCommandBuffer[9] == 0x4B and gpsCommandBuffer[10] == 0xFFFFFFCF ) {
    DEBUG_PRINT("Bottom Left");
    Keyboard.press(KEY_DOWN_ARROW);
    Keyboard.press(KEY_LEFT_ARROW);
    Keyboard.releaseAll();
  }
  DEBUG_PRINT("\r\n");
}

void bufferReset() {
  for (uint8_t i = 0U; i < nibblesReceived; i++) {
    inNibblesBuffer[i] = 0U;
  }

  nibblesReceived = 0;
  biteShiftMask = NIBBLE_RESET_BIT_POS;
}

void collectInputData() {
  uint32_t elapsed_time = 0;

  // calculate pulse time
  elapsed_time = micros() - rx_time_us;
  rx_time_us = micros();
  rx_time_ms = millis();

  if (digitalRead(UNIT_AUDIO_PIN) == LOW) {
    return;
  }

  if ( (elapsed_time > BIT_LOW_LEVEL_DURATION_MIN) && (elapsed_time < BIT_LOW_LEVEL_DURATION_MAX) ) {
    inNibblesBuffer[nibblesReceived] |= biteShiftMask;
  }

  biteShiftMask >>= 1U;

  if (biteShiftMask == 0U) {
    biteShiftMask = NIBBLE_RESET_BIT_POS; //save one nibble to one byte
    ++nibblesReceived;
  }

  if (nibblesReceived >= IN_BUFFER_SIZE) {
    DEBUG_PRINT("Buffer overflow, reset!\r\n");
    bufferReset();
  }
}

void dataPullLow(uint8_t pin){
  pinMode(pin, OUTPUT);
  digitalWrite(pin, LOW);
}
void dataFloat(uint8_t pin){
  pinMode(pin, INPUT);
}

static void send_nibble(const uint8_t message) {
  for(int i=0; i<4; i++){    //send bits 3-0
    if(message & 1<<(3-i) ){  //if bit to transmit is a 1
      dataPullLow(UNIT_AUDIO_PIN);  //pull data pin low
      delayMicroseconds(1700); //wait for appointed time
      dataFloat(UNIT_AUDIO_PIN);      //let bus go high again
      delayMicroseconds(1200);  //wait for appointed time
    }
    else{      //if bit to transmit is a 0
      dataPullLow(UNIT_AUDIO_PIN);  //pull data pin low
      delayMicroseconds(500); //wait for appointed time
      dataFloat(UNIT_AUDIO_PIN);      //let bus go high again
      delayMicroseconds(2400);  //wait for appointed time
    }
  }
}

// Send a message on the Mazda radio bus
void send_message(const uint8_t *message, const uint8_t lenght) {
  DEBUG_PRINT("TX AUDIO[");
  DEBUG_PRINT(lenght);
  DEBUG_PRINT("] ");

  for (int i = 0; i < lenght; i++) {
    DEBUG_PRINT(((uint8_t*)message)[i], HEX);
  }

  DEBUG_PRINT("\r\n");

  noInterrupts(); {

    do {
      delay(10);
    } while (digitalRead(UNIT_AUDIO_PIN) != HIGH);

    detachInterrupt(digitalPinToInterrupt(UNIT_AUDIO_PIN));
    digitalWrite(UNIT_AUDIO_PIN, HIGH);
    pinMode(UNIT_AUDIO_PIN, OUTPUT);

    for (uint8_t i = 0; i < lenght; i++) {
      send_nibble(message[i]);
    }

    pinMode(UNIT_AUDIO_PIN, UNIT_AUDIO_PIN_INPUT_MODE);
    attachInterrupt(digitalPinToInterrupt(UNIT_AUDIO_PIN), collectInputData, CHANGE);

  } interrupts();
}

void return_to_normal_mode()
{
  send_message(TAPECMD_PLAYING, sizeof(TAPECMD_PLAYING));
  delay(7);
  send_message(TAPECMD_PLAYBACK, sizeof(TAPECMD_PLAYBACK));
}

// This brings the radio back into a known state after one of the seek
// buttons is pressed
void fast_seek_response(const uint8_t *message, const uint8_t lenght)
{
    send_message(TAPECMD_SEEKING, sizeof(TAPECMD_SEEKING));
    delay(8);
    send_message(message, lenght);
    delay(8);
    send_message(TAPECMD_SEEKING, sizeof(TAPECMD_SEEKING));
    delay(8);
    return_to_normal_mode();
}


void process_radio_message(const rxMessage_t *message) {
  //check target, 0 is tape desk
  if (message->target != Target_TapeDesk) {
    return;
  }

  switch (message->command) {
    case Command_AnyBodyHome:
      DEBUG_PRINT("Any body home msg\r\n");
      send_message(TAPECMD_POWER_ON, sizeof(TAPECMD_POWER_ON));
      delay(8);
      send_message(TAPECMD_CASSETE_PRESENT, sizeof(TAPECMD_CASSETE_PRESENT));
      break;
    case Command_WakeUp:
      DEBUG_PRINT("Wake up msg\r\n");
      send_message(TAPECMD_CASSETE_PRESENT, sizeof(TAPECMD_CASSETE_PRESENT));
      delay(10);
      send_message(TAPECMD_STOPPED, sizeof(TAPECMD_STOPPED));
      Keyboard.write(MEDIA_PLAY_PAUSE);
      break;
    case Command_Control:
      // Extract the specific subcommand and command
      uint8_t subCmd = ((message->data[1] << 4U) & 0xF0) | (message->data[2] & 0x0F);
      
      if (message->data[0] == SubCommand_Playback) {
        if (subCmd == Playback_Play) {
          DEBUG_PRINT("Playback MSG = Playback_Play\r\n");
          return_to_normal_mode();
        } else if (subCmd == Playback_FF) {
          DEBUG_PRINT("Playback MSG = Playback_FF\r\n");
          send_message(TAPECMD_SEEKING, sizeof(TAPECMD_SEEKING));
          Keyboard.write(MEDIA_NEXT);
          return_to_normal_mode();
        } else if (subCmd == Playback_REW) {
          DEBUG_PRINT("Playback MSG = Playback_REW\r\n");
          send_message(TAPECMD_SEEKING, sizeof(TAPECMD_SEEKING));
          Keyboard.write(MEDIA_PREV);
          return_to_normal_mode();
        } else if (subCmd == Playback_Stop) {
          DEBUG_PRINT("Playback MSG = Playback_Stop\r\n");
          send_message(TAPECMD_STOPPED, sizeof(TAPECMD_STOPPED));
          Keyboard.write(MEDIA_PAUSE);
        } else {
          DEBUG_PRINT("Playback MSG = ");
          DEBUG_PRINT(subCmd);
          DEBUG_PRINT("\r\n");
        }
      } else if (message->data[0] == SubCommand_SeekTrack) {
        DEBUG_PRINT("SubCommand_SeekTrack\r\n");
      } else if (message->data[0] == SubCommand_SetConfig) {
        if ( subCmd == SetConfig_RepeatMode) {
          DEBUG_PRINT("SetConfig_RepeatMode\r\n");
          return_to_normal_mode();
        } else if ( subCmd == SetConfig_RandomMode) {
          DEBUG_PRINT("SetConfig_RandomMode\r\n");
          send_message(TAPECMD_PLAYBACK, sizeof(TAPECMD_PLAYBACK));
        } else if ( subCmd == SetConfig_FastForwarding) {
          DEBUG_PRINT("SetConfig_FastForwarding\r\n");
          Keyboard.write(MEDIA_NEXT);
          fast_seek_response(TAPECMD_FAST_FORWARD, sizeof(TAPECMD_FAST_FORWARD));
        } else if ( subCmd == SetConfig_FastRewinding ) {
          DEBUG_PRINT("SetConfig_FastRewinding\r\n");
          Keyboard.write(MEDIA_PREV);
          fast_seek_response(TAPECMD_FAST_REWIND,  sizeof(TAPECMD_FAST_REWIND));
        } else {
          DEBUG_PRINT("SubCommand_SetConfig = ");
          DEBUG_PRINT(subCmd);
          DEBUG_PRINT("\r\n");
        }
      } else {
        DEBUG_PRINT("UNCKNOWN Sub command\r\n");
      }
      break;
    default:
      DEBUG_PRINT("another cmd = ");
      DEBUG_PRINT(message->command);
      DEBUG_PRINT("\r\n");
      break;
  }
}
