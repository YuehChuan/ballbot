#ifdef ENABLE_RUNTIME
void runtime()
{
  //test code end
  nowtime = micros();
  dtime = nowtime - oldtime;

  // wait a second so as not to send massive amounts of data
  //delay(1000);
  // Toggle LED
  //digitalWrite( 13, digitalRead( 13 ) ^ 1 );
  oldtime = micros();
}

void printRunTime()
{
  Serial.print(F("Run Time: "));
  Serial.print(dtime);
  Serial.print(F(" micros; "));
  Serial.print(dtime / 1000);
  Serial.print(F(" millis"));
  Serial.println();
  Serial.println();
}
#endif
