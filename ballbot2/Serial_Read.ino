void serialEvent() {
  switch (Serial.read())
  {
    case 'h':
      Serial.print("'p'\t tune p\n");
      Serial.print("'i'\t tune i\n");
      Serial.print("'d'\t tune d\n");
      Serial.print("'x'\t x\n");
      Serial.print("'y'\t y\n");
      Serial.print("'z'\t z\n");
      Serial.print("'w'\t move forward\n");
      Serial.print("'a'\t turn left\n");
      Serial.print("'s'\t move back\n");
      Serial.print("'d'\t turn right\n");
      Serial.print("'q'\t stop\n");
      break;
    case 'p':
      read_p = Serial.parseFloat();
      Serial.print("p = ");
      Serial.println(read_p);
      myPID0.SetTunings(read_p, read_i, read_d);
      break;
    case 'i':
      read_i = Serial.parseFloat();
      Serial.print("i = ");
      Serial.println(read_i);
      myPID0.SetTunings(read_p, read_i, read_d);
      break;
    case 'd':
      read_d = Serial.parseFloat();
      Serial.print("d = ");
      Serial.println(read_d);
      myPID0.SetTunings(read_p, read_i, read_d);
      break;
    case 'x':
      setMotorSpeed[0] = Serial.parseFloat();
      setMotorSpeed[0]=1.4236*setMotorSpeed[0];
      Serial.print("x = ");
      //Serial.println(setMotorSpeed[0]);
      break;
    case 'y':
      setMotorSpeed[1] = Serial.parseFloat();
      setMotorSpeed[1]=1.4236*setMotorSpeed[1];
      Serial.print("y = ");
      //Serial.println(setMotorSpeed[1]);
      break;
    case 'z':
      setMotorSpeed[2] = Serial.parseFloat();
      setMotorSpeed[2]=1.4236*setMotorSpeed[2];
      //setMotorSpeed[2] = 0.0357*setMotorSpeed[2]*setMotorSpeed[2] - 2.5419*setMotorSpeed[2] + 68.695;
      Serial.print("z = ");
      //Serial.println(setMotorSpeed[2]);
      break;
    case 'r':
      software_Reset();
      break;
  }
}
