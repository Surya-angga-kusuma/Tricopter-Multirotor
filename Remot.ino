void remote_init()
{
  pinMode(12, INPUT_PULLUP);
  pinMode(11, INPUT_PULLUP);
  pinMode(16, INPUT_PULLUP);
  pinMode(15, INPUT_PULLUP);
  pinMode(14, INPUT_PULLUP);
  pinMode(13, INPUT_PULLUP);
  attachInterrupt(12, rollris, RISING);
  attachInterrupt(11, pitchris, RISING);
  attachInterrupt(16, throtris, RISING);
  attachInterrupt(15, yawris, RISING);
  attachInterrupt(14, ch5ris, RISING);
  attachInterrupt(13, ch6ris, RISING);
}

void pitchris() 
{
  attachInterrupt(11, pitchfall, FALLING);
  channel1 = micros();
}
 
void pitchfall() 
{
  attachInterrupt(11, pitchris, RISING);
  pitch_channel = micros()-channel1;
}

void throtris() 
{
  attachInterrupt(16, throtfall, FALLING);
  channel2 = micros();
}
 
void throtfall() 
{
  attachInterrupt(16, throtris, RISING);
  throttle_channel = micros()-channel2;
}

void rollris() 
{
  attachInterrupt(12, rollfall, FALLING);
  channel3 = micros();
}
 
void rollfall() 
{
  attachInterrupt(12, rollris, RISING);
  roll_channel = micros()-channel3;
}

void yawris() 
{
  attachInterrupt(15, yawfall, FALLING);
  channel4 = micros();
}
 
void yawfall() 
{
  attachInterrupt(15, yawris, RISING);
  yaw_channel = micros()-channel4;
}

void ch5ris() 
{
  attachInterrupt(14, ch5fall, FALLING);
  channel5 = micros();
}
 
void ch5fall() 
{
  attachInterrupt(14, ch5ris, RISING);
  ch5_channel = micros()-channel5;
}

void ch6ris() 
{
  attachInterrupt(13, ch6fall, FALLING);
  channel6 = micros();
}
 
void ch6fall() 
{
  attachInterrupt(13, ch6ris, RISING);
  ch6_channel = micros()-channel5;
}

void mapremote()
{
  roll_input  = 0;
  yaw_input   = 0;
  pitch_input = 0;
  
  if (roll_channel > 1507){roll_input=roll_channel-1507;}
  else if(roll_channel < 1487){roll_input=roll_channel-1487;}  
  roll_input =constrain(roll_input,-489,489); roll_input /= 10.9;
  
  if (pitch_channel > 1507){pitch_input=pitch_channel-1507;}
  else if(pitch_channel < 1487){pitch_input=pitch_channel-1487;} 
  pitch_input =constrain(pitch_input,-486,486); pitch_input /= 10.8;
  
  if (yaw_channel > 1504){yaw_input=yaw_channel-1504;}
  else if(yaw_channel < 1484){yaw_input=yaw_channel-1484;} 
  yaw_input =constrain(yaw_input,-491,491); yaw_input /= 7.01;
  
  if(ch5_channel<1400){ch5=0;}
  if(ch5_channel>1400){ch5=1;}
}
