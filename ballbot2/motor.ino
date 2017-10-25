#ifdef ENABLE_MOTOR
void motor_setup() {

  sbi(TCCR5B, CS50);// 16MHz/1
  cbi(TCCR5B, CS51);
  cbi(TCCR5B, CS52);

  sbi(TCCR5B, WGM53);//fast pwm
  sbi(TCCR5B, WGM52);
  sbi(TCCR5A, WGM51);
  cbi(TCCR5A, WGM50);
  sbi(TCCR5A, COM5A1);//open pin 46
  sbi(TCCR5A, COM5B1);//open pin 45
  sbi(TCCR5A, COM5C1);//open pin 44
  ICR5 = 255;
  OCR5A = 0;
  OCR5B = 0;
  OCR5C = 0;

  //pinMode(statpin, OUTPUT);
  for (int i = 0; i < 3; i++)
  {
    pinMode(inApin[i], OUTPUT);
    pinMode(inBpin[i], OUTPUT);
    pinMode(pwmpin[i], OUTPUT);
    pinMode(enpin[i], OUTPUT);
  }
  for (int i = 0; i < 3; i++)
  {
    digitalWrite(inApin[i], LOW);
    digitalWrite(inBpin[i], LOW);
    digitalWrite(enpin[i], HIGH);
  }
}

void motor_vector()
{
  // 0.33333 *
  //setMotorSpeed[0]<74
  //setMotorSpeed[1]<85
  //setMotorSpeed[2]<255
  //mspeed[0] = constrain((setMotorSpeed[2] + 2 / cos(alpha) * (setMotorSpeed[0] * cos(beta) - setMotorSpeed[1] * sin(beta))), -255, 255);
  //mspeed[1] = constrain((setMotorSpeed[2] + 1 / cos(alpha) * (sin(beta) * (-1.732 * setMotorSpeed[0] + setMotorSpeed[1]) - cos(beta) * (setMotorSpeed[0] + 1.732 * setMotorSpeed[1]))), -255, 255);
  //mspeed[2] = constrain((setMotorSpeed[2] + 1 / cos(alpha) * (sin(beta) * (1.732 * setMotorSpeed[0] + setMotorSpeed[1]) + cos(beta) * (-setMotorSpeed[0] + 1.732 * setMotorSpeed[1]))), -255, 255);

mspeed[0]=sin(a_a)*cos(b_a)*setMotorSpeed[0]+sin(a_a)*sin(b_a)*setMotorSpeed[1]+cos(a_a)*setMotorSpeed[2];
mspeed[1]=sin(a_a)*cos(b_a+deg120)*setMotorSpeed[0]+sin(a_a)*sin(b_a+deg120)*setMotorSpeed[1]+cos(a_a)*setMotorSpeed[2];
mspeed[2]=sin(a_a)*cos(b_a-deg120)*setMotorSpeed[0]+sin(a_a)*sin(b_a-deg120)*setMotorSpeed[1]+cos(a_a)*setMotorSpeed[2];


  //mspeed[0] = setMotorSpeed[0] * cos(alpha) + setMotorSpeed[2] * sin(alpha);
  //mspeed[1] = setMotorSpeed[0] * cos(alpha) * cos(beta) - setMotorSpeed[1] * cos(alpha) * sin(beta) + setMotorSpeed[2] * sin(alpha);
  //mspeed[2] = -setMotorSpeed[0] * cos(alpha) * cos(beta) - setMotorSpeed[1] * cos(alpha) * sin(beta) + setMotorSpeed[2] * sin(alpha);
  mspeed[0] = constrain(mspeed[0], -255, 255);
  mspeed[1] = constrain(mspeed[1], -255, 255);
  mspeed[2] = constrain(mspeed[2], -255, 255);

  for (int i = 0; i < 3; i++)
  {
    if (mspeed[i] > 0)
    {
      motorGo(i, CW, mspeed[i]);
    }
    else if (mspeed[i] < 0)
    {
      motorGo(i, CCW, abs(mspeed[i]));
    }
    else
    {
      motorGo(i, BRAKEVCC, 0);
    }
    //Serial.println(mspeed[i]);
  }
}


void motorGo(uint8_t motor, uint8_t direct, uint8_t pwm)         //Function that controls the variables: motor(0 ou 1), direction (cw ou ccw) e pwm (entra 0 e 255);
{
  if (motor <= 2)
  {
    if (direct <= 4)
    {
      if (direct <= 1)
        digitalWrite(inApin[motor], HIGH);
      else
        digitalWrite(inApin[motor], LOW);

      if ((direct == 0) || (direct == 2))
        digitalWrite(inBpin[motor], HIGH);
      else
        digitalWrite(inBpin[motor], LOW);

      //analogWrite(pwmpin[motor], pwm);
      switch (motor)
      {
        case 0:
          motor1 = pwm;
          break;
        case 1:
          motor2 = pwm;
          break;
        case 2:
          motor3 = pwm;
          break;
      }
    }
  }


  /*if (analogRead(cspin[motor]) > CS_THRESHOLD)     // If the motor locks, it will shutdown and....

      {                                                                      // ...Resets the process of increasing the PWM
       Serial.println(analogRead(cspin[motor]));
       motorOff(motor);


      }*/
}


void motorOff(int motor)     //Function to shut the motor down case it locks
{

  for (int i = 0; i < 3; i++)
  {
    digitalWrite(inApin[i], LOW);
    digitalWrite(inBpin[i], LOW);
  }
  //analogWrite(pwmpin[motor], 0);
  switch (motor)
  {
    case 0:
      motor1 = 0;
      break;
    case 1:
      motor2 = 0;
      break;
    case 2:
      motor3 = 0;
      break;
  }
  //i=0;
  //digitalWrite(13, HIGH);
  Serial.println("Motor Locked");
  //delay(1000);
}

void motorPrint()
{
  Serial.print("setMotorSpeed:\t");
  Serial.print(setMotorSpeed[0], 0);
  Serial.print("\t");
  Serial.print(setMotorSpeed[1], 0);
  Serial.print("\t");
  Serial.println(setMotorSpeed[2], 0);
  Serial.print("real pwm:\t");
  Serial.print(mspeed[0]);
  Serial.print("\t");
  Serial.print(mspeed[1]);
  Serial.print("\t");
  Serial.println(mspeed[2]);
}
#endif
