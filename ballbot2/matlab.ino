#ifdef  ENABLE_MATLAB
void serialEvent1()
{
  //Serial1.flush();
  software_Reset();
}


void establishContact()
{
  while (Serial1.available() <= 0)
  {
    Serial1.println(0);   // send a capital A
    Serial.println(F("wait for matlab"));
    delay(100);
  }
}

void matlabPrint()
{
  realTime = millis() - startTime;
  Serial1.println(realTime);
  Serial1.println(setMotorSpeedinRad[0]);
  Serial1.println(setMotorSpeedinRad[1]);
  Serial1.println(setMotorSpeedinRad[2]);
  Serial1.println(mousexyz[0]);
  Serial1.println(mousexyz[1]);
  Serial1.println(mousexyz[2]);
}
#endif
