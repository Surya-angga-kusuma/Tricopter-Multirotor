void compass_init()
{   
  while (!compass.begin())
  {
    Serial.println("Could not find a valid HMC5883L sensor, check wiring!");
    delay(500);
  }

  compass.setRange(HMC5883L_RANGE_1_3GA);
  compass.setMeasurementMode(HMC5883L_CONTINOUS);
  compass.setDataRate(HMC5883L_DATARATE_30HZ);
  compass.setSamples(HMC5883L_SAMPLES_8);
  compass.setOffset(0, 0);
}

void compass_update()
{
  //compass_compentation();
  Vector norm   = compass.readNormalize();
  float heading = atan2(norm.YAxis, norm.XAxis);
  float declinationAngle = (21.0 + (26.0 / 60.0)) / (180 / M_PI);
  heading += declinationAngle;

  if (heading < 0)
  {
    heading += 2 * PI;
  }

  if (heading > 2 * PI)
  {
    heading -= 2 * PI;
  }
  
  headingDegrees = (heading * 180/M_PI);
  
  heading_reference = setHeading - headingDegrees;
  if(heading_reference >   180) { heading_reference -= 360;}
  if(heading_reference < - 180) { heading_reference += 360;}
  heading_control = heading_reference;
}

//void compass_compentation()
//{
//  Vector norm = compass.readNormalize();
//  acc_x = ax*2.0 / 32768.0;
//  acc_y = ay*2.0 / 32768.0;
//
//  acc_x = constrain(acc_x, -1, 1);
//  acc_y = constrain(acc_y, -1, 1);
//
//  compensatePitch = asin(-acc_x);
//  compensateRoll  = asin(acc_y);
//
//  cosComRoll  = cos(compensateRoll);
//  sinComRoll  = sin(compensateRoll);
//  cosComPitch = cos(compensatePitch);
//  sinComPitch = sin(compensatePitch);
//
//  Xh = norm.ZAxis*sinComPitch + norm.XAxis*cosComPitch;
//  Yh = norm.YAxis*sinComRoll*sinComPitch + norm.YAxis*cosComRoll - norm.ZAxis*sinComRoll*cosComPitch;
//}
