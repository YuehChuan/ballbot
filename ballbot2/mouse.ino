#ifdef ENABLE_MOUSE
void mouse_init()
{
  Serial.println(F("mouse_init start (takes about 1 second)"));
  mouse1.write(0xff);  // reset
  mouse1.read();  // ack byte
  mouse1.read();  // blank */
  mouse1.read();  // blank */
  mouse1.write(0xf0);  // remote mode
  mouse1.read();  // ack
  Serial.println(F("mouse 1 done"));

  mouse2.write(0xff);  // reset
  mouse2.read();  // ack byte
  mouse2.read();  // blank */
  mouse2.read();  // blank */
  mouse2.write(0xf0);  // remote mode
  mouse2.read();  // ack
  Serial.println(F("mouse 2 done"));

  mouse3.write(0xff);  // reset
  mouse3.read();  // ack byte
  mouse3.read();  // blank */
  mouse3.read();  // blank */
  mouse3.write(0xf0);  // remote mode
  mouse3.read();  // ack
  Serial.println(F("mouse 3 done"));
  //delayMicroseconds(100);
  Serial.println(F("mouse_init done"));
}


void mouse_read()
{
  char mstat;
  //char mx;
  //char my;


  /* get a reading from the mouse */
  mouse1.write(0xeb);  // give me data!
  mouse1.read();      // ignore ack
  mstat = mouse1.read();
  mouse_vx[0] = mouse1.read();
  mouse_vy[0] = mouse1.read();
  //mouse_x[0] += mouse_vx[0];
  //mouse_y[0] += mouse_vy[0];


  mouse2.write(0xeb);  // give me data!
  mouse2.read();      // ignore ack
  mstat = mouse2.read();
  mouse_vx[1] = mouse2.read();
  mouse_vy[1] = mouse2.read();
  //mouse_x[1] += mouse_vx[1];
  //mouse_y[1] += mouse_vy[1];

  mouse3.write(0xeb);  // give me data!
  mouse3.read();      // ignore ack
  mstat = mouse3.read();
  mouse_vx[2] = mouse3.read();
  mouse_vy[2] = mouse3.read();
  //mouse_x[2] += mouse_vx[2];
  //mouse_y[2] += mouse_vy[2];


mouseV[0]=(cos(b_s)/(sin(a_s)*(1-cos(deg120))))*mouse_vy[0]+(1/(2*sin(a_s)*sin(deg120)*(1-cos(deg120))))*((sin(b_s-deg120)-sin(b_s))*mouse_vy[1]+(sin(b_s)-sin(b_s+deg120))*mouse_vy[2]);
mouseV[1]=(sin(b_s)/(sin(a_s)*(1-cos(deg120))))*mouse_vy[0]+(1/(2*sin(a_s)*sin(deg120)*(1-cos(deg120))))*((-cos(b_s-deg120)+cos(b_s))*mouse_vy[1]+(-cos(b_s)+cos(b_s+deg120))*mouse_vy[2]);
mouseV[2]=(mouse_vy[0]+mouse_vy[1]+mouse_vy[2])/(3*cos(a_a));

  //mousexyz[0] = -(2 * mouse_y[0] - mouse_y[1] - mouse_y[2]) / (2 * cos(alphaMouse) * (1 + cos(betaMouse)));
  //mousexyz[1] = -(mouse_y[1] - mouse_y[2]) / (2 * cos(alphaMouse) * sin(betaMouse));
  //mousexyz[2] = (mouse_y[0] + mouse_y[1] + mouse_y[2]) / (2 * sin(alphaMouse) * (1 + cos(betaMouse)));

  for (int i; i < 3; i++)
  {
    mouseV[i]=Filter(mouseV[i],i);
    mousexyz[i] += mouseV[i];
  }
}

void mousePrint()
{
  int dvTime = 4;
  Serial.print("mousexyz:\t");
  Serial.print(mousexyz[0], 0);
  Serial.print("\t");
  Serial.print(mousexyz[1], 0);
  Serial.print("\t");
  Serial.print(mousexyz[2], 0);
  Serial.println();

/*
  Serial.print("mouse_y:\t");
  Serial.print(mouse_y[0]);
  Serial.print("\t");
  Serial.print(mouse_y[1]);
  Serial.print("\t");
  Serial.print(mouse_y[2]);
  Serial.println();
  
        send the data back up
        Serial.print(mstat, BIN);
        Serial.print("\tX=");
        Serial.print(mouse_x[0], DEC);
        Serial.print("\tY=");
        Serial.print(mouse_y[0], DEC);
        Serial.print("\tX=");
        Serial.print(mouse_x[1], DEC);
        Serial.print("\tY=");
        Serial.print(mouse_y[1], DEC);
        Serial.print("\tX=");
        Serial.print(mouse_x[2], DEC);
        Serial.print("\tY=");
        Serial.print(mouse_y[2], DEC);*/

}
#endif
