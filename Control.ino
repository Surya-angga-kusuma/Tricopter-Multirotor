void control()
{
	// ========== roll control ==========
	errorRoll = roll_deg - roll_input;
	ControllerIRoll += GainIroll * (errorRoll);
	if (ControllerIRoll > maxController){ ControllerIRoll = maxController; }
	else if (ControllerIRoll < maxController * -1){ ControllerIRoll = maxController * -1; }

  ControlRoll = (GainProll * errorRoll) + ControllerIRoll + (GainDroll * (gx*-1));
//  ControlRoll = GainProll * errorRoll + ControllerIRoll + GainDroll * (errorRoll - ControllerDRollLast);
//  ControlRoll = 0;
  
//	if (ControlRoll > maxController){ ControlRoll = maxController; }
//	else if (ControlRoll < maxController * -1){ ControlRoll = maxController * -1; }
	ControllerDRollLast = errorRoll;

	// ========== pitch control ==========
	errorPitch = (-1*pitch_input) - pitch_deg;
	ControllerIPitch += GainIpitch * (errorPitch);
	if (ControllerIPitch > maxController){ ControllerIPitch = maxController; }
	else if (ControllerIPitch < maxController * -1){ ControllerIPitch = maxController * -1; }

  ControlPitch = (GainPpitch * errorPitch) + ControllerIPitch + (GainDpitch*gy) ;
//	ControlPitch = GainPpitch * (errorPitch/1.321940560625) + ControllerIPitch + GainDpitch * ((errorPitch - ControllerDPitchLast)/1.321940560625);
//  ControlPitch = 0;
 
//	if (ControlPitch > maxController){ ControlPitch = maxController; }
//	else if (ControlPitch < maxController * -1){ ControlPitch = maxController * -1; }
	ControllerDPitchLast = errorPitch;

	// ========== yaw control ==========
	errorYaw = yaw_input - heading_reference;
	ControllerIYaw += GainIyaw * (errorYaw);
	if (ControllerIYaw > maxController){ ControllerIYaw = maxController; }
	else if (ControllerIYaw < maxController * -1){ ControllerIYaw = maxController * -1; }
 
	ControlYaw = GainPyaw * errorYaw + ControllerIYaw + (GainDyaw*gyro_yaw_input);
//  ControlYaw = GainPyaw * errorYaw + ControllerIYaw + (GainDyaw*gz);
//  ControlYaw = GainPyaw * errorYaw + ControllerIYaw + GainDyaw * (errorYaw - ControllerDYawLast);
//  ControlYaw = 0;
 
//	if (ControlYaw > maxController){ ControlYaw = maxController; }
//	else if (ControlYaw < maxController * -1){ ControlYaw = maxController * -1; }
	ControllerDYawLast = errorYaw;
}

void control_update()
{
  if(ch5==0)
  {
    setHeading           = heading ;
    heading_control      = 0;
    throttle             = 1000;
    ControllerIRoll      = 0;
    ControllerIPitch     = 0;
    ControllerIYaw       = 0;
    ControllerDRollLast  = 0;
    ControllerDPitchLast = 0;
    ControllerDYawLast   = 0;
    
    servo    = servoAngleInit;
    motor1   = 1000;
    motor2   = 1000;
    motor3   = 1000;
  }

  if(ch5==1)
  {
    throttle_input  = throttle_channel;
    
    if (throttle_input > 1800)
		{
			throttle_input = 1800;
		}
   
    control();
    
    motor1            = throttle_input - ControlRoll + (ControlPitch*2/3) ; //RIGHT
    motor2            = throttle_input + ControlRoll + (ControlPitch*2/3) ; //LEFT
    motor3            = throttle_input - (ControlPitch*3/3);                //REAR
    ControlYaw_Filter = (ControlYaw_Filter*0.92)+(ControlYaw*0.02);
    servo             = constrain(servoAngleInit + (ControlYaw_Filter*-1), (servoAngleInit-50), (servoAngleInit+50));


    if (motor1 < 1065){ motor1 = 1065; }
		if (motor2 < 1065){ motor2 = 1065; }
		if (motor3 < 1065){ motor3 = 1065; }

//    if (motor1 < 1065){ motor1 = 0; }
//    if (motor2 < 1065){ motor2 = 0; }
//    if (motor1 > 1065){ motor1 = 0; }
//    if (motor2 > 1065){ motor2 = 0; }
//    if (motor3 < 1065){ motor3 = 0; }
//    if (motor3 > 1065){ motor3 = 0; }

		if (motor1 > 2000){ motor1 = 2000; }
		if (motor2 > 2000){ motor2 = 2000; }
		if (motor3 > 2000){ motor3 = 2000; }
  }

  motor_update();
  update_servo();
}
