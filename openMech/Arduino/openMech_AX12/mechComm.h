#ifndef mechComm_h
#define mechComm_h

/****************************************************************************************/
/* Communication Function Prototyping                                                   */
/****************************************************************************************/
void  HostRx();             /* Host Communications Serial Rx                            */
void  ParseDFRobotComm();   /* Parse Host Communication for DFRobot Format              */
void  ParseDFRobotMsg();    /* Parse Host Message for DFRobot Format                    */
void  ParseLE3DComm();      /* Parse Host Communication for Logitech 3D Extreme Format  */
void  ParseLE3DMsg();       /* Parse Host Message for Logitech Extreme 3D Format        */
void  MechTmTx();           /* Mech Telemetry Transmit                                  */
/****************************************************************************************/
/* Host Receive Function                                                                */
/****************************************************************************************/
void HostRx() 
{
  /* HOSTCOMM_HEADER1: Base Header                                                      */
  /* HOSTCOMM_HEADER2: DFRobot Gamepad Controller Header Designator                     */
  /* HOSTCOMM_HEADER3: Logitech Extreme 3D Gamepad Controller Header Designator         */
  byte commHostByte;
    
  /* Read Serial if Available */
  if (Serial3.available() > 0) {
    commHostByte = Serial3.read();
//    SerialUSB.println(commHostByte);
    comRxHostBuffer.push(commHostByte);
  }

  /* Check Buffer Head Against Header1, If not equal, Strip Buffer Head */
  if((comRxHostBuffer.size() >= 1) && (comRxHostBuffer[0] != HOSTCOMM_HEADER1))
  {
    comRxHostBuffer.shift();
  } 
  /* Check Buffer Head+1 Against Header2 or Header3, if not equal, Strip Buffer Head */
  else if( (comRxHostBuffer.size() == 2) && !((comRxHostBuffer[1] == HOSTCOMM_HEADER2) || (comRxHostBuffer[1] == HOSTCOMM_HEADER3)) )
  {
    comRxHostBuffer.shift();
  }

  /* Call DFRobot Comunication Parse Routine */
  if( (comRxHostBuffer.size() >= sizeRxDFRobotMsg) &&  (comRxHostBuffer[0] == HOSTCOMM_HEADER1) 
          &&  (comRxHostBuffer[1] == HOSTCOMM_HEADER2) )
  {  
//    SerialUSB.print("Break DFRB\n");
    ParseDFRobotComm();
  }
  
  /* Call Logitech Extreme 3D Comunication Parse Routine */
  else if( (comRxHostBuffer.size() >= sizeRxLE3DMsg) &&  (comRxHostBuffer[0] == HOSTCOMM_HEADER1) 
          &&  (comRxHostBuffer[1] == HOSTCOMM_HEADER3) )
  {  
//    SerialUSB.print("Break LE3D\n");
    ParseLE3DComm();
  }
  else
  {
  }
}
/****************************************************************************************/
/* Construct DFRobot Controller Communication Message                                   */
/****************************************************************************************/
void ParseDFRobotComm() { 
  int i;
  int commRxChksum;
  
  
    /* Transfer Buffer to Message */
    for (int i=0; i < sizeRxDFRobotMsg; i++)
    {
      dfrbGV.commRxMsg[i] = comRxHostBuffer[i];
    }
     
    /* Verify Checksum */
    commRxChksum = 0;
    for (i = 2; i <= sizeRxDFRobotMsg - 2; i++) 
    {
      commRxChksum = commRxChksum + (unsigned int)dfrbGV.commRxMsg[i];
    }
    commRxChksum = 255 - (commRxChksum & 0xFF); 
    
    /* If Checksums Equal, Strip Buffer of Message and Parse */
    if(commRxChksum == comRxHostBuffer[sizeRxDFRobotMsg-1])
    {
      for (int i=0; i < sizeRxDFRobotMsg; i++){
        comRxHostBuffer.shift();
      }
      mechGV.commWDT = 0;
      ParseDFRobotMsg();
    }
    
    /* If Checksums not Equal, Strip Buffer Head */
    else
    { 
      comRxHostBuffer.shift();
    }
}
/****************************************************************************************/
/* Parse DFRobot Controller Message                                                     */
/****************************************************************************************/
void ParseDFRobotMsg() {
  unsigned i = 0;  
  
  // Parse Buttons
  dfrbGV.Buttons1     = unsigned(dfrbGV.commRxMsg[2]);
  dfrbGV.Buttons2     = unsigned(dfrbGV.commRxMsg[3]); 
  dfrbGV.buttonJ2     = bitRead(dfrbGV.Buttons1, 0); // J2      Right Joystick Z-Axis (Pressed = 0, Unpressed = 1023)
  dfrbGV.buttonJ1     = bitRead(dfrbGV.Buttons1, 1); // J1      Left Joystick Z-Axis (Pressed = 0, Unpressed = 1023)
  dfrbGV.buttonSelect = bitRead(dfrbGV.Buttons1, 2); // S1      Select Button
  dfrbGV.buttonStart  = bitRead(dfrbGV.Buttons1, 3); // S2      Start Button
  dfrbGV.buttonUp     = bitRead(dfrbGV.Buttons1, 4); // UP      Left Button Up
  dfrbGV.buttonLeft   = bitRead(dfrbGV.Buttons1, 5); // LEFT    Left Button Left
  dfrbGV.buttonDown   = bitRead(dfrbGV.Buttons1, 6); // DOWN    Left Button Down
  dfrbGV.buttonRight  = bitRead(dfrbGV.Buttons1, 7); // RIGHT   Left Button Right

  dfrbGV.button1      = bitRead(dfrbGV.Buttons2, 0); // 1       Right Button 1
  dfrbGV.button4      = bitRead(dfrbGV.Buttons2, 1); // 4       Right Button 4
  dfrbGV.button2      = bitRead(dfrbGV.Buttons2, 2); // 2       Right Button 2
  dfrbGV.button3      = bitRead(dfrbGV.Buttons2, 3); // 3       Right Button 3
  dfrbGV.buttonRZ1    = bitRead(dfrbGV.Buttons2, 4); // RZ1     Right Z1 Button (Upper)
  dfrbGV.buttonRZ2    = bitRead(dfrbGV.Buttons2, 5); // RZ2     Right Z2 Button (Lower)
  dfrbGV.buttonLZ1    = bitRead(dfrbGV.Buttons2, 6); // LZ1     Left Z1 Button  (Upper)
  dfrbGV.buttonLZ2    = bitRead(dfrbGV.Buttons2, 7); // LZ2     Left Z2 Button  (Lower)
  
  /* Parse Joystick Commands */ 
  dfrbGV.LeftJoystickUD  = JoystickReshape(int(dfrbGV.commRxMsg[6])  + int(dfrbGV.commRxMsg[7]  << 8));
  dfrbGV.LeftJoystickLR  = JoystickReshape(int(dfrbGV.commRxMsg[4])  + int(dfrbGV.commRxMsg[5]  << 8));
  dfrbGV.RightJoystickUD = JoystickReshape(int(dfrbGV.commRxMsg[10]) + int(dfrbGV.commRxMsg[11] << 8));
  dfrbGV.RightJoystickLR = JoystickReshape(int(dfrbGV.commRxMsg[8])  + int(dfrbGV.commRxMsg[9]  << 8));  

  /* Apply Joystick Deadzones */
  if( (dfrbGV.LeftJoystickUD < dfrbGV.JoyStickDZ) && (dfrbGV.LeftJoystickUD > -dfrbGV.JoyStickDZ) )
  {dfrbGV.LeftJoystickUD = 0.0F;}
  if( (dfrbGV.LeftJoystickLR < dfrbGV.JoyStickDZ) && (dfrbGV.LeftJoystickLR > -dfrbGV.JoyStickDZ) )
  {dfrbGV.LeftJoystickLR = 0.0F;}
  if( (dfrbGV.RightJoystickUD < dfrbGV.JoyStickDZ) && (dfrbGV.RightJoystickUD > -dfrbGV.JoyStickDZ) )
  {dfrbGV.RightJoystickUD = 0.0F;}
  if( (dfrbGV.RightJoystickLR < dfrbGV.JoyStickDZ) && (dfrbGV.RightJoystickLR > -dfrbGV.JoyStickDZ) )
  {dfrbGV.RightJoystickLR = 0.0F;}

  /* Gait State Changes */
  if      (dfrbGV.button1 == 1) mechGV.gaitCmdState = 1;           
  else if (dfrbGV.button2 == 1) mechGV.gaitCmdState = 2;           
  else if (dfrbGV.button3 == 1) mechGV.gaitCmdState = 3;           
  else if (dfrbGV.button4 == 1) mechGV.gaitCmdState = 4;    

  /* Calculate Body Height Rate Multiplier */
  mechGV.bodyHeightRateMult   = mechGV.bodyHeightRateMag*(float(dfrbGV.buttonUp)-float(dfrbGV.buttonDown));

  /* Calculate Body Pitch Rate Multiplier */
  if((dfrbGV.buttonRZ1 == 1) && (dfrbGV.buttonRZ2 == 0)){
    mechGV.bodyPitchRateMult  =  mechGV.bodyPitchRateMag * dfrbGV.RightJoystickUD;
  }
  else{
    mechGV.bodyPitchRateMult  =  0;
    }
  

  /* Calculate Body Yaw Rate Multiplier */
  mechGV.bodyYawRateMult      =  mechGV.bodyYawRateMag  * dfrbGV.RightJoystickLR;
  
  /* Calculate Body x-rate Step Multiplier */
  if (dfrbGV.LeftJoystickUD>0) 
  {
    mechGV.gaitDelxMult       =  mechGV.gaitDelxFMag    * dfrbGV.LeftJoystickUD;
  }
  else
  {
    mechGV.gaitDelxMult       =  mechGV.gaitDelxBMag    * dfrbGV.LeftJoystickUD;
  }
  
  /* Calculate Body y-rate Step Multiplier */
  mechGV.gaitDelyMult         =  mechGV.gaitDelyMag     * dfrbGV.buttonLeft - 
                                 mechGV.gaitDelyMag     * dfrbGV.buttonRight;

  /* Calculate Body yaw-rate Step Multiplier */
  mechGV.gaitDelyawMult       = -mechGV.gaitDelyawMag   * dfrbGV.LeftJoystickLR;

  
  /* Gun Instructions                                                                   */
  if(dfrbGV.buttonStart == 1){
    mechGV.gunCommInstruction = COMM_ARM;    
  }
  // Reset to Home Conditions
  else if(dfrbGV.buttonSelect == 1){
    mechGV.gunCommInstruction = COMM_DISARM;
    mechGV.bodyHeight      = 0;
    mechGV.bodyPitch       = 0;
    mechGV.bodyYaw         = 0;
    mechGV.turretYaw       = 512;
    mechGV.turretPitch     = 512;    
//    mechGV.turretYaw       = 2047;
//    mechGV.turretPitch     = 2047;
    mechGV.gaitStepHeight  = mechGV.gaitStepHeightDefault;
    mechGV.gaitDelxFMag    =  0.3F;
    mechGV.gaitDelxBMag    =  0.3F; 
    initTurret();   
  }
  else if(dfrbGV.buttonLZ1 == 1){
    mechGV.gunCommInstruction = COMM_FIRE;
  }
//  else if((dfrbGV.buttonLZ1) == 1 && (dfrbGV.buttonLZ2 == 1)){
//    mechGV.gunCommInstruction = COMM_BURST;
//  }  
//  else if((dfrbGV.buttonLZ1 == 1) || (dfrbGV.buttonLZ2 == 1)){
//    mechGV.gunCommInstruction = COMM_FIRE;
//  }
  else if((dfrbGV.buttonRZ1 == 1) && dfrbGV.buttonRZ2 == 1){
    mechGV.gunCommInstruction = COMM_HOPPER_JAM;
  }
  else if(dfrbGV.buttonRZ2 == 1){
    mechGV.gunCommInstruction = COMM_HOPPER_LOAD;
  }
  else
  {
  }    

  // Melee Weapon Engage
  if(dfrbGV.buttonLZ2 == 1 && mechGV.meleeState == 0 && mechGV.meleeStatePrev == 0){
    mechGV.meleeState = 1;
  }
  else if(dfrbGV.buttonLZ2 == 1 && mechGV.meleeState == 1 && mechGV.meleeStatePrev == 0){
    mechGV.meleeState = 0;
  }
  mechGV.meleeStatePrev = dfrbGV.buttonLZ2;
  
}
/****************************************************************************************/
/* Construct Logitech Extreme 3D Controller Communication Message                       */
/****************************************************************************************/
void ParseLE3DComm() { 
  int i;
  int commRxChksum;

//  SerialUSB.println(1);
  
    /* Transfer Buffer to Message */
    for (int i=0; i < sizeRxLE3DMsg; i++)
    {
      le3dGV.commRxMsg[i] = comRxHostBuffer[i];
    }
     
    /* Verify Checksum */
    commRxChksum = 0;
    for (i = 2; i <= sizeRxLE3DMsg - 2; i++) 
    {
      commRxChksum = commRxChksum + (unsigned int)le3dGV.commRxMsg[i];
    }
    commRxChksum = 255 - (commRxChksum & 0xFF); 

//    SerialUSB.print(commRxChksum);
//    SerialUSB.print('\t');
//    SerialUSB.print(comRxHostBuffer[sizeRxLE3DMsg-1]);
//    SerialUSB.print('\n');
    
    /* If Checksums Equal, Strip Buffer of Message and Parse */
    if(commRxChksum == comRxHostBuffer[sizeRxLE3DMsg-1])
    {
      for (int i=0; i < sizeRxLE3DMsg; i++){
        comRxHostBuffer.shift();
      }
      mechGV.commWDT = 0;
      ParseLE3DMsg();
    }
    
    /* If Checksums not Equal, Strip Buffer Head */
    else
    { 
      comRxHostBuffer.shift();
    }
}
/****************************************************************************************/
/* Parse DFRobot Controller Message                                                     */
/****************************************************************************************/
void ParseLE3DMsg() {
//  unsigned i = 0;  

  le3dGV.ButtonsA       = le3dGV.commRxMsg[2];
  le3dGV.ButtonsB       = le3dGV.commRxMsg[3];
  le3dGV.ButtonsC       = le3dGV.commRxMsg[4];
  le3dGV.JoystickXLo    = le3dGV.commRxMsg[5];
  le3dGV.JoystickXHi    = le3dGV.commRxMsg[6];
  le3dGV.JoystickYLo    = le3dGV.commRxMsg[7];
  le3dGV.JoystickYHi    = le3dGV.commRxMsg[8];
  le3dGV.JoystickTwist  = le3dGV.commRxMsg[9];
  le3dGV.JoystickHat    = le3dGV.commRxMsg[10];
  le3dGV.JoystickSlider = le3dGV.commRxMsg[11];
  le3dGV.Spare12        = le3dGV.commRxMsg[12];
  le3dGV.Spare13        = le3dGV.commRxMsg[13];
  le3dGV.Spare14        = le3dGV.commRxMsg[14];
  le3dGV.Spare15        = le3dGV.commRxMsg[15];
  le3dGV.Spare16        = le3dGV.commRxMsg[16];
  le3dGV.Spare17        = le3dGV.commRxMsg[17];
  le3dGV.Spare18        = le3dGV.commRxMsg[18];
  le3dGV.Spare19        = le3dGV.commRxMsg[19];
  le3dGV.Spare20        = le3dGV.commRxMsg[20];
  le3dGV.Spare21        = le3dGV.commRxMsg[21];
  le3dGV.Spare22        = le3dGV.commRxMsg[22];
  le3dGV.Spare23        = le3dGV.commRxMsg[23];

  le3dGV.Trigger        = bitRead(le3dGV.ButtonsA,0);
  le3dGV.Trigger2       = bitRead(le3dGV.ButtonsA,1);
  le3dGV.LLJoyButton    = bitRead(le3dGV.ButtonsA,2);
  le3dGV.LRJoyButton    = bitRead(le3dGV.ButtonsA,3);
  le3dGV.ULJoyButton    = bitRead(le3dGV.ButtonsA,4);
  le3dGV.URJoyButton    = bitRead(le3dGV.ButtonsA,5);
  le3dGV.ULBaseButton   = bitRead(le3dGV.ButtonsA,6);
  le3dGV.URBaseButton   = bitRead(le3dGV.ButtonsA,7);

  le3dGV.MLBaseButton   = bitRead(le3dGV.ButtonsB,0);
  le3dGV.MRBaseButton   = bitRead(le3dGV.ButtonsB,1);
  le3dGV.LLBaseButton   = bitRead(le3dGV.ButtonsB,2);
  le3dGV.URBaseButton   = bitRead(le3dGV.ButtonsB,3);
  le3dGV.SpareButton    = bitRead(le3dGV.ButtonsB,4);
  le3dGV.Spare11Button  = bitRead(le3dGV.ButtonsB,5);
  le3dGV.Spare10Button  = bitRead(le3dGV.ButtonsB,6);
  le3dGV.Spare9Button   = bitRead(le3dGV.ButtonsB,7);

  le3dGV.Spare8Button   = bitRead(le3dGV.ButtonsC,0);
  le3dGV.Spare7Button   = bitRead(le3dGV.ButtonsC,1);
  le3dGV.Spare6Button   = bitRead(le3dGV.ButtonsC,2);
  le3dGV.Spare5Button   = bitRead(le3dGV.ButtonsC,3);
  le3dGV.Spare4Button   = bitRead(le3dGV.ButtonsC,4);
  le3dGV.Spare3Button   = bitRead(le3dGV.ButtonsC,5);
  le3dGV.Spare2Button   = bitRead(le3dGV.ButtonsC,6);
  le3dGV.Spare1Button   = bitRead(le3dGV.ButtonsC,7);

  /* Parse Joystick Commands */ 
  le3dGV.JoystickX =  JoystickReshape(int(le3dGV.JoystickXLo) + int(le3dGV.JoystickXHi << 8));
  le3dGV.JoystickY = -JoystickReshape(int(le3dGV.JoystickYLo) + int(le3dGV.JoystickYHi << 8)); 
  
  /* Apply Joystick Deadzones */
  if( (le3dGV.JoystickX < le3dGV.JoyStickDZ) && (le3dGV.JoystickX > -le3dGV.JoyStickDZ) )
  {le3dGV.JoystickX = 0.0F;}
  if( (le3dGV.JoystickY < le3dGV.JoyStickDZ) && (le3dGV.JoystickY > -le3dGV.JoyStickDZ) )
  {le3dGV.JoystickY = 0.0F;}
//  SerialUSB.println( le3dGV.JoystickY );
  
  /* Gait State Changes */
  if      (le3dGV.ULJoyButton == 1) mechGV.gaitCmdState = 4;           
  else if (le3dGV.LLJoyButton == 1) mechGV.gaitCmdState = 3;           
  else if (le3dGV.LRJoyButton == 1) mechGV.gaitCmdState = 2;           
  else if (le3dGV.URJoyButton == 1) mechGV.gaitCmdState = 1; 

///* Calculate Body Height Rate Multiplier */
//  mechGV.bodyHeightRateMult   = mechGV.bodyHeightRateMag*(float(dfrbGV.buttonUp)-float(dfrbGV.buttonDown));
//
//  /* Calculate Body Pitch Rate Multiplier */
//  if((dfrbGV.buttonRZ1 == 1) && (dfrbGV.buttonRZ2 == 0)){
//    mechGV.bodyPitchRateMult  =  mechGV.bodyPitchRateMag * dfrbGV.RightJoystickUD;
//  }
//
  /* Calculate Body Yaw Rate Multiplier */
  mechGV.bodyYawRateMult      =  mechGV.bodyYawRateMag  * (float(le3dGV.JoystickTwist)-127.0F)/127.0;

  /* Calculate Body x-rate Step Multiplier */
  if (le3dGV.JoystickY > 0) 
  {
    mechGV.gaitDelxMult       =  mechGV.gaitDelxFMag    * le3dGV.JoystickY;
  }
  else
  {
    mechGV.gaitDelxMult       =  mechGV.gaitDelxBMag    * le3dGV.JoystickY;
  }
  
  /* Calculate Body y-rate Step Multiplier */
  if(le3dGV.Trigger2 == 1 && le3dGV.JoystickX != 0)
  {
    mechGV.gaitDelyMult       =  mechGV.gaitDelyMag     * le3dGV.JoystickX;
  }
  else
  {
    mechGV.gaitDelyMult = 0.0F;
  }
  
  /* Calculate Body yaw-rate Step Multiplier */
  if(le3dGV.Trigger2 == 0 && le3dGV.JoystickX != 0)
  {
    mechGV.gaitDelyawMult     = mechGV.gaitDelyawMag   * le3dGV.JoystickX;
  }       
  else
  {
    mechGV.gaitDelyawMult = 0.0F;
  }
}

/****************************************************************************************/
/* Dynamixel Transmit Function                                                          */
/****************************************************************************************/
void GaitTx() { 
//  // Write to Dynamixels using Synch Write - Fast but confusing to understand (1 msec)
//  byte SyncPage[60];   
//  SyncPage[0]  = L11;
//  SyncPage[1]  = lowByte(mechGV.uTheta0[0]);
//  SyncPage[2]  = highByte(mechGV.uTheta0[0]); 
//  SyncPage[3]  = 0;
//  SyncPage[4]  = 0;
//  SyncPage[5]  = L12;
//  SyncPage[6]  = lowByte(mechGV.uTheta0[1]);
//  SyncPage[7]  = highByte(mechGV.uTheta0[1]); 
//  SyncPage[8]  = 0;
//  SyncPage[9]  = 0;
//  SyncPage[10] = L13;
//  SyncPage[11] = lowByte(mechGV.uTheta0[2]);
//  SyncPage[12] = highByte(mechGV.uTheta0[2]);
//  SyncPage[13] = 0;
//  SyncPage[14] = 0;
//  SyncPage[15] = L21;
//  SyncPage[16] = lowByte(mechGV.uTheta1[0]);
//  SyncPage[17] = highByte(mechGV.uTheta1[0]);
//  SyncPage[18] = 0;
//  SyncPage[19] = 0;
//  SyncPage[20] = L22;
//  SyncPage[21] = lowByte(mechGV.uTheta1[1]);
//  SyncPage[22] = highByte(mechGV.uTheta1[1]);
//  SyncPage[23] = 0;
//  SyncPage[24] = 0;
//  SyncPage[25] = L23;
//  SyncPage[26] = lowByte(mechGV.uTheta1[2]);
//  SyncPage[27] = highByte(mechGV.uTheta1[2]);
//  SyncPage[28] = 0;
//  SyncPage[29] = 0;
//  SyncPage[30] = L31;
//  SyncPage[31] = lowByte(mechGV.uTheta2[0]);
//  SyncPage[32] = highByte(mechGV.uTheta2[0]);
//  SyncPage[33] = 0;
//  SyncPage[34] = 0;
//  SyncPage[35] = L32;
//  SyncPage[36] = lowByte(mechGV.uTheta2[1]);
//  SyncPage[37] = highByte(mechGV.uTheta2[1]);
//  SyncPage[38] = 0;
//  SyncPage[39] = 0;
//  SyncPage[40] = L33;
//  SyncPage[41] = lowByte(mechGV.uTheta2[2]);
//  SyncPage[42] = highByte(mechGV.uTheta2[2]);
//  SyncPage[43] = 0;
//  SyncPage[44] = 0;
//  SyncPage[45] = L41;
//  SyncPage[46] = lowByte(mechGV.uTheta3[0]);
//  SyncPage[47] = highByte(mechGV.uTheta3[0]);
//  SyncPage[48] = 0;
//  SyncPage[49] = 0;
//  SyncPage[50] = L42;
//  SyncPage[51] = lowByte(mechGV.uTheta3[1]);
//  SyncPage[52] = highByte(mechGV.uTheta3[1]);
//  SyncPage[53] = 0;
//  SyncPage[54] = 0;
//  SyncPage[55] = L43;
//  SyncPage[56] = lowByte(mechGV.uTheta3[2]);
//  SyncPage[57] = highByte(mechGV.uTheta3[2]);
//  SyncPage[58] = 0;
//  SyncPage[59] = 0;
//  X430.SynchWrite(ADDR_X430_GOAL_POSITION, 4, SyncPage, 60);


//  unsigned int uSpeed   = 1023;                // Dynamixel Speed Command
//  word SyncPage[36]; 
//
//  SyncPage[0]  = L11;
//  SyncPage[1]  = mechGV.uTheta0[0];
//  SyncPage[2]  = uSpeed;
//  SyncPage[3]  = L12;
//  SyncPage[4]  = mechGV.uTheta0[1];
//  SyncPage[5]  = uSpeed;
//  SyncPage[6]  = L13;
//  SyncPage[7]  = mechGV.uTheta0[2];
//  SyncPage[8]  = uSpeed;
//
//  SyncPage[9]  = L21;
//  SyncPage[10] = mechGV.uTheta1[0];
//  SyncPage[11] = uSpeed;
//  SyncPage[12] = L22;
//  SyncPage[13] = mechGV.uTheta1[1];
//  SyncPage[14] = uSpeed;
//  SyncPage[15] = L23;
//  SyncPage[16] = mechGV.uTheta1[2];
//  SyncPage[17] = uSpeed;
//
//  SyncPage[18] = L31;
//  SyncPage[19] = mechGV.uTheta2[0];
//  SyncPage[20] = uSpeed;
//  SyncPage[21] = L32;
//  SyncPage[22] = mechGV.uTheta2[1];
//  SyncPage[23] = uSpeed;
//  SyncPage[24] = L33;
//  SyncPage[25] = mechGV.uTheta2[2];
//  SyncPage[26] = uSpeed;
//
//  SyncPage[27] = L41;
//  SyncPage[28] = mechGV.uTheta3[0];
//  SyncPage[29] = uSpeed;
//  SyncPage[30] = L42;
//  SyncPage[31] = mechGV.uTheta3[1];
//  SyncPage[32] = uSpeed;
//  SyncPage[33] = L43;
//  SyncPage[34] = mechGV.uTheta3[2];
//  SyncPage[35] = uSpeed;
//
//  AX12.SynchWrite(30,2,SyncPage,36);
  
  AX12.GoalPosition(L11,mechGV.uTheta0[0]);
  AX12.GoalPosition(L12,mechGV.uTheta0[1]);
  AX12.GoalPosition(L13,mechGV.uTheta0[2]);
  
  AX12.GoalPosition(L21,mechGV.uTheta1[0]);
  AX12.GoalPosition(L22,mechGV.uTheta1[1]);
  AX12.GoalPosition(L23,mechGV.uTheta1[2]);
  
  AX12.GoalPosition(L31,mechGV.uTheta2[0]);
  AX12.GoalPosition(L32,mechGV.uTheta2[1]);
  AX12.GoalPosition(L33,mechGV.uTheta2[2]);
  
  AX12.GoalPosition(L41,mechGV.uTheta3[0]); 
  AX12.GoalPosition(L42,mechGV.uTheta3[1]);
  AX12.GoalPosition(L43,mechGV.uTheta3[2]);
}

/****************************************************************************************/
/* Transmit TM Buffer                                                                   */
/****************************************************************************************/
void  MechTmTx()
{
  Serial3.write(TM_HEADER1);
  Serial3.write(TM_HEADER2);
  unsigned int commTmChksum = 0;

  byte MechTmStates1        = byte(mechGV.gaitState);
  byte MechTmStates2        = byte(mechGV.gunCommInstruction);
  byte MechTmStates3        = byte(0);
  byte MechTmTargetPlates   = byte(0);

  MechTmTargetPlates = mechGV.TargetFrontFlag + mechGV.TargetLeftFlag*2 + mechGV.TargetRearFlag*4 + mechGV.TargetRightFlag*8;
  if(MechTmTargetPlates != 0){
    SerialUSB.println(MechTmTargetPlates);
  }

  comTmBuffer[0]  = lowByte(mechGV.powrMainCnts);     // Main Battery Voltage Low Byte  (cnts)
  comTmBuffer[1]  = highByte(mechGV.powrMainCnts);    // Main Battery Voltage High Byte (cnts)
  comTmBuffer[2]  = lowByte(0);                       // Auxiliary Battery Voltage Low Byte  (cnts)
  comTmBuffer[3]  = highByte(0);                      // Auxiliary Battery Voltage High Byte (cnts)
  comTmBuffer[4]  = MechTmStates1;                    //
  comTmBuffer[5]  = MechTmStates2;                    //            
  comTmBuffer[6]  = MechTmStates3;                    //
  comTmBuffer[7]  = MechTmTargetPlates;               // 
  comTmBuffer[8]  = lowByte(mechGV.uTheta0[0]);
  comTmBuffer[9]  = highByte(mechGV.uTheta0[0]);
  comTmBuffer[10] = lowByte(mechGV.uTheta0[1]);
  comTmBuffer[11] = highByte(mechGV.uTheta0[1]);
  comTmBuffer[12] = lowByte(mechGV.uTheta0[2]);
  comTmBuffer[13] = highByte(mechGV.uTheta0[2]);
  comTmBuffer[20] = 20;
  comTmBuffer[21] = 21;
  comTmBuffer[22] = 22;
  comTmBuffer[23] = 23;
  comTmBuffer[24] = 24;
  comTmBuffer[25] = 25;
  comTmBuffer[26] = 26;
  comTmBuffer[27] = 27;
  comTmBuffer[28] = lowByte(mechGV.turretPitch);
  comTmBuffer[29] = highByte(mechGV.turretPitch);
  
  for (int i = 0; i < sizeTmBuffer; i++) {
    Serial3.write(comTmBuffer[i]);
    commTmChksum = commTmChksum + (unsigned int)comTmBuffer[i];
  }
  commTmChksum = 255 - (commTmChksum & 0xFF); 
  Serial3.write(commTmChksum);
}

/****************************************************************************************/
/* End Mech Comm Header                                                                 */
/****************************************************************************************/
#endif
