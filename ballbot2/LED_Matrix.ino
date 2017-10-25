#ifdef ENABLE_LED
void LED_setup() {
  //we have already set the number of devices when we created the LedControl
  int devices = lc.getDeviceCount();
  //we have to init all devices in a loop
  for (int address = 0; address < devices; address++) {
    /*The MAX72XX is in power-saving mode on startup*/
    lc.shutdown(address, false);
    /* Set the brightness to a medium values */
    lc.setIntensity(address, 8);
    /* and clear the display */
    lc.clearDisplay(address);
  }
  lc.setDigit(1, 0, 0, false);
  lc.setDigit(1, 1, 0, false);
  lc.setDigit(1, 2, 0, false);
  lc.setDigit(1, 3, 0, true);
  lc.setDigit(1, 4, 0, false);
  lc.setDigit(1, 5, 0, false);
  lc.setDigit(1, 6, 0, true);
  lc.setDigit(1, 7, 0, false);

  lc.setLed(0, 3, 3, true);
  lc.setLed(0, 3, 4, true);
  lc.setLed(0, 4, 3, true);
  lc.setLed(0, 4, 4, true);
}

void LED_control() {
  unsigned long displayTime = millis();
  lc.setDigit(1, 0, displayTime % 10, false);
  lc.setDigit(1, 1, displayTime % 100 / 10, false);
  lc.setDigit(1, 2, displayTime % 1000 / 100, false);
  lc.setDigit(1, 3, displayTime % 10000 / 1000, true);
  lc.setDigit(1, 4, displayTime % 100000 / 10000, false);
  lc.setDigit(1, 5, displayTime % 1000000 / 100000, false);
  lc.setDigit(1, 6, displayTime % 10000000 / 1000000, true);
  lc.setDigit(1, 7, displayTime % 100000000 / 10000000, false);


  int ledx = map(roll, -10, 10, 0, 6);
  int ledy = map(pitch, -10, 10, 0, 6);
  lc.clearDisplay(0);
  lc.setLed(0, ledx, ledy, true);
  lc.setLed(0, ledx + 1, ledy, true);
  lc.setLed(0, ledx, ledy + 1, true);
  lc.setLed(0, ledx + 1, ledy + 1, true);
}
#endif
