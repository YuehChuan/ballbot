void PID_setup(){
  //initialize the variables we're linked to
  Input[0] = 0;
  Setpoint[0] = 0;
  Input[1] = 0;
  Setpoint[1] = 0;
  Input[2] = 0;
  Setpoint[2] = 0;

  //turn the PID on
  myPID0.SetMode(AUTOMATIC);
  myPID0.SetOutputLimits(-255, 255);
  myPID0.SetSampleTime(PIDSampleTime);

  myPID1.SetMode(AUTOMATIC);
  myPID1.SetOutputLimits(-255, 255);
  myPID1.SetSampleTime(PIDSampleTime);

  myPID2.SetMode(AUTOMATIC);
  myPID2.SetOutputLimits(-255, 255);
  myPID2.SetSampleTime(PIDSampleTime);
}
