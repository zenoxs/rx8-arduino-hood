void togglePowerScreen() {
  Serial.println("togglePowerScreen");
  digitalWrite(SCREEN_PIN, HIGH);
  timerPowerScreen.start();
}

void turnPinScreen(int idx, int v, int up) {
  Serial.println("turnPinScreen");
  digitalWrite(SCREEN_PIN, LOW);
}
