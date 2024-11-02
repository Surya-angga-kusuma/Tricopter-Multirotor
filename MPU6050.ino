void init_MPU()
{
  mpu.initialize();
  devStatus = mpu.dmpInitialize();

  mpu.setXGyroOffset(21);
  mpu.setYGyroOffset(40);
  mpu.setZGyroOffset(-1);
  mpu.setXAccelOffset(-1018);
  mpu.setYAccelOffset(632);
  mpu.setZAccelOffset(1569);
  if (devStatus == 0) {

    mpu.setDMPEnabled(true);
    attachInterrupt(17, dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();
    packetSize = mpu.dmpGetFIFOPacketSize();
  }

  mpu.setI2CMasterModeEnabled(false);
  mpu.setI2CBypassEnabled(false) ;
  mpu.setSleepEnabled(false);
}

void get_YPR()
{
  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  gyroX_filt = gyroX_filt * 0.93  + gx * 0.07;
  gyroY_filt = (gyroY_filt * 0.93 + gy * 0.07) * -1;
  gyroZ_filt = gyroZ_filt * 0.93  + gz * 0.07;
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();
  fifoCount = mpu.getFIFOCount();
  if ((mpuIntStatus & 0x10) || fifoCount >= 1024)
  {
    mpu.resetFIFO();
  }
  else if (mpuIntStatus & 0x02)
  {
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
    mpu.getFIFOBytes(fifoBuffer, packetSize);
    fifoCount -= packetSize;

    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

    pitch_deg = (ypr[1] * 180 / M_PI)+pitch_correction;
    roll_deg  = (ypr[2] * 180 / M_PI)+roll_correction;
    heading   = (ypr[0] * 180 / M_PI);
    
    heading_reference = setHeading - heading;
    if(heading_reference >   180) { heading_reference -= 360;}
    if(heading_reference < - 180) { heading_reference += 360;}

   
    mpu.dmpGetQuaternion(&q, fifoBuffer); mpu.dmpGetAccel(&aa, fifoBuffer); mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity); mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
  }
}
