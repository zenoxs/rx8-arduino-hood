void togglePowerScreen() {
  digitalWrite(SCREEN_PIN, HIGH);
  timerPowerScreen.start();
}

void turnPinScreen(int idx, int v, int up) {
  digitalWrite(SCREEN_PIN, LOW);
}
