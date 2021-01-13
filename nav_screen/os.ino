void startOS(int idx, int v, int up)
{
  Serial.println("START OS");
  Serial.println("POWER OS ON");
  digitalWrite(OS_POWER_RELAY_PIN, HIGH);
}

void stopOS(int idx, int v, int up)
{
  Serial.println("STOP OS");
  digitalWrite(OS_PIN, LOW);
  delay(7000); // Need to wait 7 sec to trigger turn off
  digitalWrite(OS_PIN, HIGH);
  delay(10000); // Wait 15 sec for android to turn off
  digitalWrite(OS_POWER_RELAY_PIN, LOW);
}
