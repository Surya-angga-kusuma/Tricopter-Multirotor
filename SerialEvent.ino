void serialEvent() 
{
  while (Serial2.available())
  { 
    inChar = Serial2.read();
//    if (inChar == 'a'){ servoAngleInit += 1;}
//    if (inChar == 's'){ servoAngleInit -= 1;}    
//    if (inChar == 'w'){ GainProll += 0.01;}
//    if (inChar == 'q'){ GainProll -= 0.01;}
//    if (inChar == 's'){ GainDroll += 0.0001;}
//    if (inChar == 'a'){ GainDroll -= 0.0001;}
//    if (inChar == 'x'){ GainIroll += 0.0001;}
//    if (inChar == 'z'){ GainIroll -= 0.0001;}

//    if (inChar == 'a'){ roll_correction  += 0.01;}
//    if (inChar == 's'){ roll_correction  -= 0.01;}
//    if (inChar == 'z'){ pitch_correction += 0.01;}
//    if (inChar == 'x'){ pitch_correction -= 0.01;}

    if (inChar == 'r'){ GainPpitch += 0.01;}
    if (inChar == 'e'){ GainPpitch -= 0.01;}
    if (inChar == 'f'){ GainDpitch += 0.0001;}
    if (inChar == 'd'){ GainDpitch -= 0.0001;}
    if (inChar == 'v'){ GainIpitch += 0.0001;}
    if (inChar == 'c'){ GainIpitch -= 0.0001;}
//
    if (inChar == 'y'){ GainPyaw += 0.01;}
    if (inChar == 't'){ GainPyaw -= 0.01;}
    if (inChar == 'h'){ GainDyaw += 0.0001;}
    if (inChar == 'g'){ GainDyaw -= 0.0001;}
    if (inChar == 'n'){ GainIyaw += 0.0001;}
    if (inChar == 'b'){ GainIyaw -= 0.0001;}

   }
}
