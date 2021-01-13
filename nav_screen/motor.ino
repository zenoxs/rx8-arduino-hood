void stopMotor()
{
  digitalWrite(L_EN_PIN, LOW); // Stop the motor
  digitalWrite(R_EN_PIN, LOW); // Stop the motor
  analogWrite(LPWM_PIN, 0);
  analogWrite(RPWM_PIN, 0);
}

void startMotor(bool dir)
{
  digitalWrite(L_EN_PIN, HIGH); // Enable the motor
  digitalWrite(R_EN_PIN, HIGH); // Enable the motor

  if (dir)
  {
    analogWrite(RPWM_PIN, 255);
  }
  else
  {
    analogWrite(LPWM_PIN, 255);
  }
}
