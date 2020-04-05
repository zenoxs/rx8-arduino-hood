#include <Keyboard.h>
char cmd[15];
int cmdCursor = 0;
bool commandReceived = false;

void setup() {
  Serial1.begin(2400);
  Serial.begin(9600);
  Serial.println("Setup start");
  commandReceived = false;
  Serial.println("Setup end");
  Keyboard.begin();
}

void loop() {
  if (commandReceived == true) {
    //Right Arrow
    if (cmd[6] == 0x4F and cmd[7] == 0x4B and cmd[8] == 0xCB and cmd[9] == 0x7B and cmd[10] == 0xCF ) {
      Serial.println("Right");
      Keyboard.press(KEY_RIGHT_ARROW);
      Keyboard.releaseAll();
    }
    //Left Arrow
    if (cmd[6] == 0xCF and cmd[7] == 0x4B and cmd[8] == 0x4B and cmd[9] == 0xCF and cmd[10] == 0xCF ) {
      Serial.println("Left");
      Keyboard.press(KEY_LEFT_ARROW);
      Keyboard.releaseAll();
    }
    //Up Arrow
    if (cmd[6] == 0x4F and cmd[7] == 0x4B and cmd[8] == 0xCB and cmd[9] == 0xCF and cmd[10] == 0xCF ) {
      Serial.println("Up");
      Keyboard.press(KEY_UP_ARROW);
      Keyboard.releaseAll();
    }
    //Down Arrow
    if (cmd[6] == 0xCF and cmd[7] == 0x4F and cmd[8] == 0x4B and cmd[9] == 0xCB and cmd[10] == 0xCF ) {
      Serial.println("Down");
      Keyboard.press(KEY_DOWN_ARROW);
      Keyboard.releaseAll();
    }
    // Enter Button
    if (cmd[6] == 0x4F and cmd[7] == 0x7B and cmd[8] == 0x4B and cmd[9] == 0xCF and cmd[10] == 0x7B ) {
      Serial.println("Enter");
      Keyboard.press(KEY_RETURN);
      Keyboard.press(KEY_F1);
      Keyboard.releaseAll();
    }
    // RET Button
    if (cmd[6] == 0xCF and cmd[7] == 0xCB and cmd[8] == 0x4B and cmd[9] == 0x7B and cmd[10] == 0x7B ) {
      Serial.println("Ret");
      Keyboard.press(KEY_ESC);
      Keyboard.releaseAll();
    }
    // Zoom In Button (arrow to front) need to modify navit keys.h
    if (cmd[6] == 0x4F and cmd[7] == 0x7B and cmd[8] == 0x4B and cmd[9] == 0x7B and cmd[10] == 0xCF ) {
      Serial.println("Zoom in");
      Keyboard.press(KEY_PAGE_UP);
      Keyboard.releaseAll();
    }
    // Zoom out Button (arrow to back) need to modify navit keys.h
    if (cmd[6] == 0xCF and cmd[7] == 0x4F and cmd[8] == 0x4B and cmd[9] == 0xCB and cmd[10] == 0x7B ) {
      Serial.println("Zoom out");
      Keyboard.press(KEY_PAGE_DOWN);
      Keyboard.releaseAll();
    }
    // Menu
    if (cmd[6] == 0x4F and cmd[7] == 0xCF and cmd[8] == 0x4B and cmd[9] == 0xCF and cmd[10] == 0xCB ) {
      Serial.println("Menu");
      Keyboard.press(KEY_HOME);
      Keyboard.releaseAll();
    }
    // Voice Button
    if (cmd[6] == 0x4F and cmd[7] == 0xCF and cmd[8] == 0x4B and cmd[9] == 0x4F and cmd[10] == 0x7B ) {
      Serial.println("Voice");
      Keyboard.press('v');
      Keyboard.releaseAll();
    }
    // POS Button
    if (cmd[6] == 0xCF and cmd[7] == 0x7B and cmd[8] == 0x4B and cmd[9] == 0x7B and cmd[10] == 0x7B ) {
      Serial.println("Pos");
      Keyboard.press('p');
      Keyboard.releaseAll();
    }
    // Haut Gauche
    if (cmd[6] == 0x4F and cmd[7] == 0x4F and cmd[8] == 0x4B and cmd[9] == 0x4F and cmd[10] == 0xCF ) {
      Serial.println("Top Left");
      Keyboard.press(KEY_UP_ARROW);
      Keyboard.press(KEY_LEFT_ARROW);
      Keyboard.releaseAll();
    }
    // bas droite
    if (cmd[6] == 0x4F and cmd[7] == 0x4F and cmd[8] == 0x4B and cmd[9] == 0xCB and cmd[10] == 0xCF ) {
      Serial.println("Bottom Right");
      Keyboard.press(KEY_DOWN_ARROW);
      Keyboard.press(KEY_RIGHT_ARROW);
      Keyboard.releaseAll();
    }
    // haut droite
    if (cmd[6] == 0xCF and cmd[7] == 0x7B and cmd[8] == 0x4B and cmd[9] == 0x7B and cmd[10] == 0xCF ) {
      Serial.println("Top Right");
      Keyboard.press(KEY_UP_ARROW);
      Keyboard.press(KEY_RIGHT_ARROW);
      Keyboard.releaseAll();
    }
    // bas gauche
    if (cmd[6] == 0xCF and cmd[7] == 0xCF and cmd[8] == 0x4B and cmd[9] == 0x4B and cmd[10] == 0xCF ) {
      Serial.println("Bottom Left");
      Keyboard.press(KEY_DOWN_ARROW);
      Keyboard.press(KEY_LEFT_ARROW);
      Keyboard.releaseAll();
    }
    for (int i = 0; i <= 10; i++) {
      Serial.print(cmd[i], HEX);
      cmd[i] = 0;
      commandReceived = false;
    }
    Serial.println("");
  }
}

void serialEvent1() {
  while (Serial1.available()) {
    char inChar = (char)Serial1.read();
    if (inChar == 0) {
      cmdCursor = 0;
    }
    cmd[cmdCursor] = inChar;
    cmdCursor++;
    if (cmdCursor > 12) {
      commandReceived = true;
      cmdCursor = 0;
    }
  }
}
