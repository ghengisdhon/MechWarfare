#ifndef mechUtilities_h
#define mechUtilities_h

/****************************************************************************************/
/* Mech Function Prototypes                                                             */
/****************************************************************************************/
void  ChkBattery();
float JoystickReshape(int x);
void  mechGaitControl();
void  mechTurretControl();
void  mechBodyControl();
void  mechGunControl();
void  GunTx();
void  initTurret();
void  TurretTx();  
int   AZTurretReshape(float x);
int   ELTurretReshape(float x);

// SPI Ports Setup  
// SPI1 Port - Not Used
SPIClass SPI_1(1);
// SPI2 Port - Gun Communications
SPIClass SPI_2(2);

/****************************************************************************************/
/* Check Battery Voltage                                                                */
/****************************************************************************************/
void ChkBattery()
{
    mechGV.powrMainCnts  = analogRead(pinPowrMain);  
    mechGV.powrMainVolt  = float(mechGV.powrMainCnts)*0.0131912568306011;

    //SerialUSB.print(mechGV.powrMainVolt);
    //SerialUSB.print('\n');
}

/****************************************************************************************/  
/****************************************************************************************/  
/****************************************************************************************/  
/*                                                                                      */
/* Gait Functions                                                                       */
/*                                                                                      */
/****************************************************************************************/  
/****************************************************************************************/  
/****************************************************************************************/

/****************************************************************************************/
/* Gait Change                                                                          */
/****************************************************************************************/
void mechGaitControl()
{
  /**************************************************************************************/
  /* Set Gait Interrupt Update Speed (How Fast Gait Steps are Executed)                 */
  /* Set Step Size Multiplier                                                           */
  /* Set Step Height Multiplier                                                         */
  /**************************************************************************************/
  if (mechGV.gaitCmdState == 1 && mechGV.gaitState !=  1) {
    Timer.pause();
    Timer.setPeriod(TimerMS1);
    Timer.refresh();
    Timer.resume();
    mechGV.gaitStepSize       =  uStepSize1;         
    mechGV.gaitState          =  1;      
    mechGV.gaitStepHeight     =  StepHeight1;
    mechGV.gaitDelxFMag       =  0.3F;
    mechGV.gaitDelxBMag       =  0.3F; 
    mechGV.gaitDelyawMag      =  0.034906585039887F;
    mechGV.bodyHeight         =  0.0F;
    mechGV.bodyHeightMax      =  1.5F;
    mechGV.bodyHeightMin      = -0.25F;
    mechGV.bodyPitchMax       = mechGV.bodyPitchMaxDefault;
    mechGV.bodyPitchMin       = mechGV.bodyPitchMinDefault;      
    mechGV.gaitStepIncrement  =  1U;
    memcpy(XFT,XFTS1,sizeXFT);
    memcpy(YFT,YFTS1,sizeYFT);
    memcpy(ZFT,ZFTS1,sizeZFT);
    memcpy(delxyLF_vec,delxyLF_S2_vec,sizedelxy);
    memcpy(delxyLR_vec,delxyLR_S2_vec,sizedelxy);
    memcpy(delxyRR_vec,delxyRR_S2_vec,sizedelxy);
    memcpy(delxyRF_vec,delxyRF_S2_vec,sizedelxy);
    memcpy(delyawLF_vec,delyawLF_S2_vec,sizedelyaw);
    memcpy(delyawLR_vec,delyawLR_S2_vec,sizedelyaw);
    memcpy(delyawRR_vec,delyawRR_S2_vec,sizedelyaw);
    memcpy(delyawRF_vec,delyawRF_S2_vec,sizedelyaw);
    memcpy(delzLF_vec,delzLF_S2_vec,sizedelz);
    memcpy(delzLR_vec,delzLR_S2_vec,sizedelz);
    memcpy(delzRR_vec,delzRR_S2_vec,sizedelz);
    memcpy(delzRF_vec,delzRF_S2_vec,sizedelz);
    memcpy(delzadj_vec,delzadj_S2_vec,sizedelzadj);    
  }
  else if (mechGV.gaitCmdState == 2  && mechGV.gaitState !=  2) {
    Timer.pause();
    Timer.setPeriod(TimerMS2);
    Timer.refresh();
    Timer.resume();
    mechGV.gaitStepSize       =  uStepSize2;      
    mechGV.gaitState          =  2;  
    mechGV.gaitStepHeight     =  StepHeight2;
    mechGV.gaitDelxFMag       =  0.3F;
    mechGV.gaitDelxBMag       =  0.3F; 
    mechGV.gaitDelyawMag      =  0.034906585039887F;
    mechGV.bodyHeight         =  0.0F;
    mechGV.bodyHeightMax      =  1.5F;
    mechGV.bodyHeightMin      = -0.25F;
    mechGV.bodyPitchMax       = mechGV.bodyPitchMaxDefault;
    mechGV.bodyPitchMin       = mechGV.bodyPitchMinDefault;    
    mechGV.gaitStepIncrement  = 1U;
    memcpy(XFT,XFTS2,sizeXFT);
    memcpy(YFT,YFTS2,sizeYFT);
    memcpy(ZFT,ZFTS2,sizeZFT);
    memcpy(delxyLF_vec,delxyLF_S2_vec,sizedelxy);
    memcpy(delxyLR_vec,delxyLR_S2_vec,sizedelxy);
    memcpy(delxyRR_vec,delxyRR_S2_vec,sizedelxy);
    memcpy(delxyRF_vec,delxyRF_S2_vec,sizedelxy);
    memcpy(delyawLF_vec,delyawLF_S2_vec,sizedelyaw);
    memcpy(delyawLR_vec,delyawLR_S2_vec,sizedelyaw);
    memcpy(delyawRR_vec,delyawRR_S2_vec,sizedelyaw);
    memcpy(delyawRF_vec,delyawRF_S2_vec,sizedelyaw);
    memcpy(delzLF_vec,delzLF_S2_vec,sizedelz);
    memcpy(delzLR_vec,delzLR_S2_vec,sizedelz);
    memcpy(delzRR_vec,delzRR_S2_vec,sizedelz);
    memcpy(delzRF_vec,delzRF_S2_vec,sizedelz);
    memcpy(delzadj_vec,delzadj_S2_vec,sizedelzadj);    
  }
  else if (mechGV.gaitCmdState == 3  && mechGV.gaitState !=  3) {
    Timer.pause();
    Timer.setPeriod(TimerMS3);
    Timer.refresh();
    Timer.resume();
    mechGV.gaitStepSize       =  uStepSize3;
    mechGV.gaitState          =  3;     
    mechGV.gaitStepHeight     =  StepHeight3;  
    mechGV.gaitDelxFMag       =  0.3F;
    mechGV.gaitDelxBMag       =  0.2F; 
    mechGV.gaitDelyawMag      =  0.034906585039887F;     
    mechGV.bodyHeight         =  0.0F;
    mechGV.bodyHeightMax      =  1.25F;
    mechGV.bodyHeightMin      = -0.25F;
    mechGV.bodyPitch          = 0;
    mechGV.bodyPitchMax       = 0;
    mechGV.bodyPitchMin       = 0;
    mechGV.gaitStepIncrement  = 2U;    
    mechGV.gaitIdx            = (mechGV.gaitIdx/2)*2;   
  
    memcpy(XFT,XFTS3,sizeXFT);
    memcpy(YFT,YFTS3,sizeYFT);
    memcpy(ZFT,ZFTS3,sizeZFT);
    memcpy(delxyLF_vec,delxyLF_S2_vec,sizedelxy);
    memcpy(delxyLR_vec,delxyLR_S2_vec,sizedelxy);
    memcpy(delxyRR_vec,delxyRR_S2_vec,sizedelxy);
    memcpy(delxyRF_vec,delxyRF_S2_vec,sizedelxy);
    memcpy(delyawLF_vec,delyawLF_S2_vec,sizedelyaw);
    memcpy(delyawLR_vec,delyawLR_S2_vec,sizedelyaw);
    memcpy(delyawRR_vec,delyawRR_S2_vec,sizedelyaw);
    memcpy(delyawRF_vec,delyawRF_S2_vec,sizedelyaw);
    memcpy(delzLF_vec,delzLF_S2_vec,sizedelz);
    memcpy(delzLR_vec,delzLR_S2_vec,sizedelz);
    memcpy(delzRR_vec,delzRR_S2_vec,sizedelz);
    memcpy(delzRF_vec,delzRF_S2_vec,sizedelz);
    memcpy(delzadj_vec,delzadj_S2_vec,sizedelzadj);    
  }
  if (mechGV.gaitCmdState == 4  && mechGV.gaitState !=  4) {
    Timer.pause();
    Timer.setPeriod(TimerMS4);
    Timer.refresh();
    Timer.resume();
    mechGV.gaitStepSize       = uStepSize4;
    mechGV.gaitState          = 4;         
    mechGV.gaitStepHeight     = StepHeight4;  
    mechGV.gaitDelxFMag       =  0.35F;
    mechGV.gaitDelxBMag       =  0.3F;    
    mechGV.gaitDelyawMag      =  0.034906585039887F*0.75F;
    mechGV.bodyHeight         =  0.00F;
    mechGV.bodyHeightMax      =  1.00F;
    mechGV.bodyHeightMin      = -0.25F;
    mechGV.bodyPitch          = 0;
    mechGV.bodyPitchMax       = 0;
    mechGV.bodyPitchMin       = 0;    
    mechGV.gaitStepIncrement  = 2U;       
    mechGV.gaitIdx            = (mechGV.gaitIdx/2)*2;
    memcpy(XFT,XFTS4,sizeXFT);
    memcpy(YFT,YFTS4,sizeYFT);
    memcpy(ZFT,ZFTS4,sizeZFT);
    memcpy(delxyLF_vec,delxyLF_S2_vec,sizedelxy);
    memcpy(delxyLR_vec,delxyLR_S2_vec,sizedelxy);
    memcpy(delxyRR_vec,delxyRR_S2_vec,sizedelxy);
    memcpy(delxyRF_vec,delxyRF_S2_vec,sizedelxy);
    memcpy(delyawLF_vec,delyawLF_S2_vec,sizedelyaw);
    memcpy(delyawLR_vec,delyawLR_S2_vec,sizedelyaw);
    memcpy(delyawRR_vec,delyawRR_S2_vec,sizedelyaw);
    memcpy(delyawRF_vec,delyawRF_S2_vec,sizedelyaw);
    memcpy(delzLF_vec,delzLF_S2_vec,sizedelz);
    memcpy(delzLR_vec,delzLR_S2_vec,sizedelz);
    memcpy(delzRR_vec,delzRR_S2_vec,sizedelz);
    memcpy(delzRF_vec,delzRF_S2_vec,sizedelz);
    memcpy(delzadj_vec,delzadj_S2_vec,sizedelzadj);      
  }  
}
/****************************************************************************************/  
/****************************************************************************************/  
/****************************************************************************************/  
/*                                                                                      */
/* Kinematics Control Functions                                                         */
/*                                                                                      */
/****************************************************************************************/  
/****************************************************************************************/  
/****************************************************************************************/
  
/****************************************************************************************/  
/* Body Kinematics Control                                                              */
/****************************************************************************************/
void mechBodyControl()
{
  /* Body Height Adjustment */
  if (mechGV.bodyHeightRateMult != 0.0F) {
    mechGV.bodyHeight =  mechGV.bodyHeight + mechGV.bodyHeightRateMult;
    mechGV.bodyHeight =  constrain(mechGV.bodyHeight,mechGV.bodyHeightMin,mechGV.bodyHeightMax);
    mechGV.bodyPitch == 0; /* Set Body Pitch to Zero if Height is Changing */
  }

  /* Body Pitch Adjustment */
  mechGV.bodyPitch = mechGV.bodyPitch + mechGV.bodyPitchRateMult;
  mechGV.bodyPitch = constrain(mechGV.bodyPitch,mechGV.bodyPitchMin,mechGV.bodyPitchMax);

  /* Overide Body Height if Body Pitch is Changing */
  if(mechGV.bodyPitch < 0){
//    mechGV.bodyHeight = mechGV.bodyHeightMax*abs(mechGV.bodyPitch)/abs(mechGV.bodyPitchMax);
    mechGV.bodyHeight = 0;
  }
  else if(mechGV.bodyPitch > 0){
//    mechGV.bodyHeight = mechGV.bodyHeightMax*abs(mechGV.bodyPitch)/abs(mechGV.bodyPitchMax);
    mechGV.bodyHeight = 0;
  }

  /* Set Height and Body Pitch to Zero while Moving Forward/Reverse */
  if(dfrbGV.LeftJoystickUD != 0.0F && mechGV.gaitCmdState != 1){
    if( abs(mechGV.bodyPitch) > 0.0){
      mechGV.bodyHeight = 0.0F;
    }
    mechGV.bodyPitch = 0.0F;
  }

  /* Step Commands in x-direction */
  mechGV.gaitDelx   = mechGV.gaitDelxMult   * float(mechGV.gaitStepSize);

  /* Step Commands in y-direction */
  mechGV.gaitDely   = mechGV.gaitDelyMult   * float(mechGV.gaitStepSize);
                      
  /* Step & Body Yaw Adjustment  */
  mechGV.gaitDelyaw = mechGV.gaitDelyawMult * float(mechGV.gaitStepSize);

  /* Check if Command is Greater than Deadzone, Check Not at Lower Limit, if not, Move Body CCW */
  if (((mechGV.bodyYaw > mechGV.bodyYawMin) && (mechGV.bodyYawRateMult > 0)) || 
      ((mechGV.bodyYaw < mechGV.bodyYawMax) && (mechGV.bodyYawRateMult < 0)))
  {
    mechGV.bodyYaw = mechGV.bodyYaw - mechGV.bodyYawRateMult;
  }
  else
  {
    mechGV.gaitDelyaw = mechGV.gaitDelyaw - 10*mechGV.bodyYawRateMult;
  }
  
}

/****************************************************************************************/  
/****************************************************************************************/  
/****************************************************************************************/  
/*                                                                                      */
/* Turret Functions                                                                     */
/*                                                                                      */
/****************************************************************************************/  
/****************************************************************************************/  
/****************************************************************************************/

/****************************************************************************************/
/* Turret Initialization                                                                */
/****************************************************************************************/
void initTurret(){
  X430.TorqueEnable(EL1,1); // Torque Enable
//  mechGV.turretYaw    = 2047;
//  mechGV.turretPitch  = 2047;
//
//  X430.StatusReturnLevel(EL1,0);
//  X430.PositionPGain(EL1,640);
//  X430.PositionIGain(EL1,0);
//  X430.PositionDGain(EL1,0);
//  X430.StatusReturnLevel(EL1,1);
//  X430.GoalPosition(EL1,mechGV.turretPitch);

  mechGV.turretYaw    = 511;
  mechGV.turretPitch  = 511;
  AX12.GoalPosition(EL1,mechGV.turretPitch);
  AX12.GoalPosition(EL2,mechGV.turretPitch);
  
}
/****************************************************************************************/
/* Turret Transmit                                                                      */
/****************************************************************************************/
void TurretTx() { 
  // X430 Based Turret
//  uint16_t AZ1posTx = mechGV.turretYaw-AZbias;
//  uint16_t AZ2posTx = mechGV.turretYaw+AZbias;
//  uint16_t EL1posTx = 4095-mechGV.turretPitch;
//  uint16_t EL2posTx = mechGV.turretPitch;
//  
  // Write to Dynamixels using Synch Write - Fast but confusing to understand (0.7 msec)
//  byte SyncPage[20]; 
//  SyncPage[0]  = AZ1;
//  SyncPage[1]  = lowByte(AZ1posTx);
//  SyncPage[2]  = highByte(AZ1posTx);
//  SyncPage[3]  = 0;
//  SyncPage[4]  = 0;
//  SyncPage[5]  = AZ2;
//  SyncPage[6]  = lowByte(AZ2posTx);
//  SyncPage[7]  = highByte(AZ2posTx);
//  SyncPage[8]  = 0;
//  SyncPage[9]  = 0;
//  SyncPage[10]  = EL1;
//  SyncPage[11]  = lowByte(EL1posTx);
//  SyncPage[12]  = highByte(EL1posTx);
//  SyncPage[13]  = 0;
//  SyncPage[14]  = 0;
//  SyncPage[15]  = EL2;
//  SyncPage[16] = lowByte(EL2posTx);
//  SyncPage[17] = highByte(EL2posTx);
//  SyncPage[18]  = 0;
//  SyncPage[19]  = 0;
//  X430.SynchWrite(ADDR_X430_GOAL_POSITION, 4, SyncPage, 20);
  
  uint16_t AZ1posTx = mechGV.turretYaw-AZbias;
  uint16_t AZ2posTx = mechGV.turretYaw+AZbias;
  uint16_t EL1posTx = mechGV.turretPitch;
  uint16_t EL2posTx = 1023-mechGV.turretPitch;
  AX12.GoalPosition(EL1,EL1posTx);
  AX12.GoalPosition(EL2,EL2posTx);

  if (mechGV.meleeState == 1){
    AX12.MovingSpeed(MELEE1,1023);
  }
  else{
    AX12.MovingSpeed(MELEE1,0);
  }
}

/****************************************************************************************/  
/* Turrent Control                                                                      */
/****************************************************************************************/
void mechTurretControl()
{
  /* Turret Pitch Adjustment                                                            */
  if (dfrbGV.RightJoystickUD < 0 && (dfrbGV.buttonRZ1 == 0) && (dfrbGV.buttonRZ2 == 0) && (mechGV.turretPitch >= ELMinLim)){
     mechGV.turretPitch = mechGV.turretPitch- ELTurretReshape(dfrbGV.RightJoystickUD);
  }
  else if (dfrbGV.RightJoystickUD > 0 && (dfrbGV.buttonRZ1 ==0)&& (dfrbGV.buttonRZ2 == 0) && (mechGV.turretPitch <= ELMaxLim)){
     mechGV.turretPitch = mechGV.turretPitch- ELTurretReshape(dfrbGV.RightJoystickUD);
  }
  else{
    mechGV.turretPitch = mechGV.turretPitch;
  }
//  if(mechGV.turretPitch < ELMinLim){mechGV.turretPitch = ELMinLim;}
//  if(mechGV.turretPitch > ELMaxLim){mechGV.turretPitch = ELMaxLim;} 
  
  /* Turret Yaw Adjustment                                                              */  
  if (dfrbGV.RightJoystickLR < 0){
     mechGV.turretYaw  = mechGV.turretYaw - AZTurretReshape(dfrbGV.RightJoystickLR);
  }
  else if (dfrbGV.RightJoystickLR > 0){
     mechGV.turretYaw  = mechGV.turretYaw - AZTurretReshape(dfrbGV.RightJoystickLR);
  }
  if(mechGV.turretYaw < AZMinLim){mechGV.turretYaw = AZMinLim;}
  if(mechGV.turretYaw > AZMaxLim){mechGV.turretYaw = AZMaxLim;} 
}

/****************************************************************************************/  
/* Reshape AZ Turret Joystick Command from -1 to +1 with Deadband at Zero               */                     
/****************************************************************************************/  
int AZTurretReshape(float x) {
  int y;
  float sign;

  if(x < 0){
    sign = -1;
  }
  else{
    sign = 1;
  }

  if(abs(x) < 0.125){
    y = 0;
  }
  else if(abs(x) < .35){
    y = sign*1;
  }
  else if(abs(x) < .85){
//    y = sign*5;
    y = sign*2;    
  }
  else{
//    y = sign*20;
  y = sign*3;
  }

  return y;
}
/****************************************************************************************/  
/* Reshape EL Turret Joystick Command from -1 to +1 with Deadband at Zero               */                     
/****************************************************************************************/  
int ELTurretReshape(float x) {
  int y;
  float sign;

  if(x < 0){
    sign = 1;
  }
  else{
    sign = -1;
  }

  if(abs(x) < 0.125){
    y = 0;
  }
  else if(abs(x) < .35){
    y = sign*1;
  }
  else if(abs(x) < .85){
//    y = sign*5;
  y = sign*1;
  }
  else{
//    y = sign*10;
  y = sign*1;
  }

  return y;
}  
  
/****************************************************************************************/  
/****************************************************************************************/  
/****************************************************************************************/  
/*                                                                                      */
/* Gun Functions                                                                        */
/*                                                                                      */
/****************************************************************************************/  
/****************************************************************************************/  
/****************************************************************************************/

/****************************************************************************************/  
/* Gun Transmit Function                                                                */
/****************************************************************************************/  
void GunTx(){ 
//  SerialUSB.println(mechGV.gunCommInstruction);
  if(mechGV.gunCommInstruction == COMM_ARM){
    SPI_2.transfer(103);
    SPI_2.transfer(117);
    SPI_2.transfer(110);
    SPI_2.transfer(65);
    SPI_2.transfer(80);
    SPI_2.transfer('\n');
    SerialUSB.println("Arm");
    mechGV.gunCommInstruction = 0;
  }
  else if(mechGV.gunCommInstruction == COMM_DISARM){
    SPI_2.transfer(103);
    SPI_2.transfer(117);
    SPI_2.transfer(110);
    SPI_2.transfer(68);
    SPI_2.transfer(77);
    SPI_2.transfer('\n');
    
    mechGV.gunCommInstruction = 0;
  }
  else if(mechGV.gunCommInstruction == COMM_FIRE){
    SPI_2.transfer(103);
    SPI_2.transfer(117);
    SPI_2.transfer(110);
    SPI_2.transfer(70);
    SPI_2.transfer(75);
    SPI_2.transfer('\n');
    
    mechGV.gunCommInstruction = 0;
  }
  else if(mechGV.gunCommInstruction == COMM_BURST){
    SPI_2.transfer(103);
    SPI_2.transfer(117);
    SPI_2.transfer(110);
    SPI_2.transfer(66);
    SPI_2.transfer(79);
    SPI_2.transfer('\n');
    
    mechGV.gunCommInstruction = 0;
  }
  else if(mechGV.gunCommInstruction == COMM_HOPPER_JAM){
    SPI_2.transfer(103);
    SPI_2.transfer(117);
    SPI_2.transfer(110);
    SPI_2.transfer(74);
    SPI_2.transfer(71);
    SPI_2.transfer('\n');
    
    mechGV.gunCommInstruction = 0;
  }  
  else if(mechGV.gunCommInstruction == COMM_HOPPER_LOAD){
    SPI_2.transfer(103);
    SPI_2.transfer(117);
    SPI_2.transfer(110);
    SPI_2.transfer(72);
    SPI_2.transfer(73);
    SPI_2.transfer('\n');
    
    mechGV.gunCommInstruction = 0;
  }  
}

/****************************************************************************************/  
/* Gun Command Control                                                                  */
/****************************************************************************************/
void mechGunControl()
{
//  /* Gun Instructions                                                                   */
//  if(mechGV.buttonStart == 1){
//    mechGV.gunCommInstruction = COMM_ARM;    
//  }
//  
//  // Reset to Home Conditions
//  else if(mechGV.buttonSelect == 1){
//    mechGV.gunCommInstruction = COMM_DISARM;
//    
//    mechGV.bodyHeight      = 0;
//    mechGV.bodyPitch       = 0;
//    mechGV.bodyYaw         = 0;
//    //mechGV.turretYaw       = 512;
//    //mechGV.turretPitch     = 512;    
//    mechGV.turretYaw       = 2047;
//    mechGV.turretPitch     = 2047;
//    mechGV.gaitStepHeight  = mechGV.gaitStepHeightDefault;
////    mechGV.gaitDelxFMult         =  0.3F;
////    mechGV.gaitDelxRMult         =  0.3F; 
//    initTurret();   
//  }
//  else if((mechGV.buttonLZ1) == 1 && (mechGV.buttonLZ2 == 1)){
//    mechGV.gunCommInstruction = COMM_BURST;
//    //mechGV.gunCommInstruction = COMM_FIRE;
//  }  
//  else if((mechGV.buttonLZ1 == 1) || (mechGV.buttonLZ2 == 1)){
//    mechGV.gunCommInstruction = COMM_FIRE;
//  }
//  else if((dfrbGV.buttonRZ1 == 1) && dfrbGV.buttonRZ2 == 1){
//    mechGV.gunCommInstruction = COMM_HOPPER_JAM;
//  }
//  else if(dfrbGV.buttonRZ2 == 1){
//    mechGV.gunCommInstruction = COMM_HOPPER_LOAD;
//  }
//  else
//  {
//  }
}

/****************************************************************************************/  
/****************************************************************************************/  
/****************************************************************************************/  
/*                                                                                      */
/* Joystick Functions                                                                   */
/*                                                                                      */
/****************************************************************************************/  
/****************************************************************************************/  
/****************************************************************************************/

/****************************************************************************************/  
/* Reshape Analog Joystick for Deadband at Zero                                         */
/****************************************************************************************/
  float JoystickReshape(int x) {
    float y;  
    y = (float(x) - 512) / 512;
    
  if (abs(y) <= .25) {
      y = 0;
    }
    return y;
  }
/****************************************************************************************/
/* End Mech Utilities Header                                                            */
/****************************************************************************************/

#endif
