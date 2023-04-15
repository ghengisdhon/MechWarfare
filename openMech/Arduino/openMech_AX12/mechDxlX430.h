#ifndef mechX430_h
#define mechX430_h
/****************************************************************************************/
/* X430 Function Prototyping                                                           */
/****************************************************************************************/

/****************************************************************************************/
/* Declare X430 (Protocol 2) Class                                                     */
/****************************************************************************************/
class DynamixelX430
{
  private:
  dynamixel::PortHandler *x430_portHandler;
  dynamixel::PacketHandler *x430_packetHandler;
  
  public:
  void begin(const char* device_name, float protocol_version, uint32_t baud_rate);
  bool setBaudrate(uint32_t baud_rate);
  void ReturnTimeDelay(uint8_t id, uint8_t data);
  void MaxPositionLimit(uint8_t id, uint16_t data);
  void MinPositionLimit(uint8_t id, uint16_t data);
  void OperatingMode(uint8_t id, uint8_t data);
  void StatusReturnLevel(uint8_t id, uint8_t data);
  void TorqueEnable(uint8_t id, uint8_t data);
  void LED(uint8_t id, uint8_t data);
  void PositionDGain(uint8_t id, uint16_t data);
  void PositionIGain(uint8_t id, uint16_t data);
  void PositionPGain(uint8_t id, uint16_t data);
  void GoalPosition(uint8_t id, uint16_t data);

  void SynchWrite(uint16_t start_address, uint16_t data_length, uint8_t *param, uint16_t param_length);
  void writeByte(uint8_t id, uint16_t address, uint8_t data);
  void writeWord(uint8_t id, uint16_t address, uint16_t data);
//  uint8_t readByte(uint8_t id, uint8_t address);
//  uint16_t readWord(uint8_t id, uint16_t address);

};

/****************************************************************************************/
/* X430 Begin                                                                          */
/****************************************************************************************/
void DynamixelX430::begin(const char* device_name, float protocol_version, uint32_t baud_rate ){
  
  // Initialize PortHandler Instance
  // Get methods and members of PortHandlerLinux or PortHandlerWindows
  x430_portHandler = dynamixel::PortHandler::getPortHandler(device_name);

  // Initialize PacketHandler Instance
  // Set the protocol version
  
  // Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
  x430_packetHandler = dynamixel::PacketHandler::getPacketHandler(protocol_version);

  // Open Port
  if (!x430_portHandler->openPort()) {
    SerialUSB.print("Failed to open the Dynamixel X430!\n");
  }

/* Baud Rate determines serial communication speed between a controller and DYNAMIXELs.

/* Value   Baud Rate   Margin of Error                                                  */
/* -----   ---------   ---------------                                                  */
/*     7   4.5M        0.000%                                                           */
/*     6   4M          0.000%                                                           */
/*     5   3M          0.000%                                                           */
/*     4   2M          0.000%                                                           */
/*     3   1M          0.000%                                                           */
/*     2   115,200     0.000%                                                           */
/*     1   57,600      0.000%  (Default)                                                */
  x430_portHandler->setBaudRate(baud_rate);
}
/****************************************************************************************/
/* X430 Return Time Delay                                                              */
/****************************************************************************************/
/* After the DYNAMIXEL receives an Instruction Packet, it delays transmitting the Status*/  
/* Packet for Return Delay Time (9). For instance, if the Return Delay Time(9) is set to*/ 
/* ‘10’, the Status Packet will be returned after 20[μsec] when the Instruction Packet  */
/* is received.                                                                         */
/*                                                                                      */
/* Unit     Value Range   Description                                                   */
/* ----     -----------   -----------                                                   */
/* 2[μsec]  0 ~ 254       Default value ‘250’(500[μsec]), Maximum 508[μsec]             */
/****************************************************************************************/
void DynamixelX430::ReturnTimeDelay(uint8_t id, uint8_t data){
  x430_packetHandler->write1ByteTxOnly(x430_portHandler, id, ADDR_X430_RETURN_TIME_DELAY, data);
}

/****************************************************************************************/
/* X430 Angle Limits                                                                   */
/****************************************************************************************/
/* The angle limit allows the motion to be restrained. The range and the unit of the    */
/* value is the same as Goal Position(Address 30, 31).                                  */
/****************************************************************************************/
void DynamixelX430::MaxPositionLimit(uint8_t id, uint16_t data){
  x430_packetHandler->write4ByteTxOnly(x430_portHandler, id, ADDR_X430_MAX_POSITION_LIMIT , data);
}

void DynamixelX430::MinPositionLimit(uint8_t id, uint16_t data){
  x430_packetHandler->write4ByteTxOnly(x430_portHandler, id, ADDR_X430_MIN_POSITION_LIMIT , data);
}
/****************************************************************************************/
/* X430 Control Mode                                                   */
/****************************************************************************************/
/* Value   Operating Mode   Description
/* -----   --------------   -----------
/*     1   Velocity        (0° ~ 360°) This mode controls velocity and ideal for wheel  */
/*                          operation. This mode is identical to the Wheel Mode(endless)*/ 
/*                          from existing Dynamixels.                                   */       
/*     3   Position        (Default) This mode controls position and identical to the   */
/*                          Joint Mode. Operating position range is limited by Max      */
/*                          Position Limit(48) and Min Position Limit(52). This mode is */
/*                          ideal for articulated robots that each joint rotates less   */
/*                          than 360°.                                                  */
/*     4  Extended Position (Multi-turn)  This mode controls position and identical to  */
/*                          Multi-turn Mode. 512 turns are supported(-256[rev]~256[rev])*/
/*                          and ideal for multi-turn wrists or conveyer systems or a    */
/*                          system that requires an additional reduction gear.          */
/*    16  PWM               (Voltage Control Mode)  This mode directly controls PWM     */
/*                          output (Voltage Control Mode)                               */
/****************************************************************************************/
void DynamixelX430::OperatingMode(uint8_t id, uint8_t data){
  x430_packetHandler->write1ByteTxOnly(x430_portHandler, id, ADDR_X430_OPERATING_MODE, data);
}
/****************************************************************************************/
/* X430 Status Return Level                                                            */
/****************************************************************************************/
/* This value decides how to return Status Packet when Dynamixel receives an            */
/* Instruction Packet.                                                                  */
/*                                                                                      */
/* Value  Responding Instructions   Description                                         */
/* -----  -----------------------   -----------                                         */
/* 0      PING Instruction          Status Packet will not be returned for all          */
/*                                  Instructions                                        */
/* 1      PING Instruction          Status  Packet will be returned only for READ       */        
/*        READ Instruction          Instruction                                         */
/*                                                                                      */
/* 2      All Instructions          Status Packet will be returned for all Instructions */
/****************************************************************************************/
void DynamixelX430::StatusReturnLevel(uint8_t id, uint8_t data){
  x430_packetHandler->write1ByteTxOnly(x430_portHandler, id, ADDR_X430_STATUS_RETURN_LEVEL, data);
}

/****************************************************************************************/
/* X430 Torque Enable                                                                  */
/****************************************************************************************/
/* Value  Description                                                                   */
/* -----  -----------                                                                   */ 
/* 0      Turn off the torque(Free run state)                                           */
/* 1      Turn on the torque and lock EEPROM area                                       */
/****************************************************************************************/
void DynamixelX430::TorqueEnable(uint8_t id, uint8_t data){
  x430_packetHandler->write1ByteTxOnly(x430_portHandler, id, ADDR_X430_TORQUE_ENABLE, data);
}

/****************************************************************************************/
/* X430 LED                                                                            */
/****************************************************************************************/
/* The combination of bit changes the output color of XL-320.                           */
/* Bit          Output Color                                                            */
/* ---          ------------                                                            */
/* 0            Turn OFF the LED (Default)                                              */
/* 1            Turn ON the LED                                                         */
/****************************************************************************************/
void DynamixelX430::LED(uint8_t id, uint8_t data){
  x430_packetHandler->write1ByteTxOnly(x430_portHandler, id, ADDR_X430_LED, data);
}

/****************************************************************************************/
/* X430 Position PID Gains                                                             */
/****************************************************************************************/
/* These Gains are used in Position Control Mode and Extended Position Control Mode.    */
/* Gains of Dynamixel’s internal controller can be calculated from Gains of the Control */ 
/* Table as shown below. The constant in each equations include sampling time. Position */
/* P Gain of Dynamixel’s internal controller is abbreviated to KPP and that of the      */
/* Control Table is abbreviated to KPP(TBL).                                            */
/*                                                                                      */
/* Controller            Gain   Conversion Equations    Range       Description         */
/* ----------            ----   --------------------    -----       -----------         */
/* Position D Gain(80)   KPD    KPD = KPD(TBL)/16       0 ~ 16,383  D Gain              */
/* Position I Gain(82)   KPI    KPI = KPI(TBL)/65,536   0 ~ 16,383  I Gain              */
/* Position P Gain(84)   KPP    KPP = KPP(TBL)/128      0 ~ 16,383  P Gain              */
/* Feedforward 2nd (88)  KFF2nd KFF2nd(TBL)/4           0 ~ 16,383  Feedforward Accel   */
/* Feedforward 1st (90)  KFF1st KFF1st(TBL)/4           0 ~ 16,383  Feedforward Vel     */
/*                                                                                      */
/* An Instruction from the user is transmitted via Dynamixel bus, then registered to    */
/* Goal Position(116). Goal Position(116) is converted to target position trajectory    */
/* and target velocity trajectory by Profile Velocity(112) and Profile Acceleration(108)*/
/* The target position trajectory and target velocity trajectory is stored at Position  */
/* Trajectory(140) and Velocity Trajectory(136) respectively. Feedforward and PID       */
/* controller calculate PWM output for the motor based on target trajectories. Goal     */
/* PWM(100) sets a limit on the calculated PWM output and decides the final PWM value.  */
/* The final PWM value is applied to the motor through an Inverter, and the horn of     */
/* Dynamixel is driven. Results are stored at Present Position(132), Present Velocity   */
/* (128), Present PWM(124) and Present Load(126).                                       */

void DynamixelX430::PositionDGain(uint8_t id, uint16_t data){

  x430_packetHandler->write2ByteTxOnly(x430_portHandler, id, ADDR_X430_POSITION_D_GAIN, data);
}

void DynamixelX430::PositionIGain(uint8_t id, uint16_t data){
  x430_packetHandler->write2ByteTxOnly(x430_portHandler, id, ADDR_X430_POSITION_I_GAIN, data);
}

void DynamixelX430::PositionPGain(uint8_t id, uint16_t data){
  x430_packetHandler->write2ByteTxOnly(x430_portHandler, id, ADDR_X430_POSITION_P_GAIN, data);
}

/****************************************************************************************/
/* X430 Goal Position                                                                  */
/****************************************************************************************/
/* Desired position can be set with Goal Position(116). From the front view of          */
/* Dynamixels, CCW is an increasing direction whereas CW is a decreasing direction. The */
/* way to reaching Goal Position(116) is differ by 4 Profiles provided by Dynamixels.   */
/* Please refer to the Profile Velocity(112) for more details.                          */
/*                                                                                      */
/* Mode                     Values                              Description             */
/* ----                     ------                              -----------             */
/* Position                 Min Pos Limit(52)~Max Pos Limit(48) Initial Value: 0~4,095  */
/* Extended Position        -1,048,575~1,048,575                -256[rev]~256[rev]      */
/* Current-based Position   -1,048,575~1,048,575                -256[rev]~256[rev]      */
/*                                                                                      */
/* Degree Conversion Constant  Description                                              */
/* 0.088°/Value  1[rev] : 0 ~ 4,095                                                     */
/*                                                                                      */
/* NOTE: If Profile Acceleration(108), Profile Velocity(112) and Goal Position(116) are */ 
/* modified simultaneously, Goal Position(116) is processed based on updated Profile    */
/* Acceleration(108) and Profile Velocity(112).                                         */
/****************************************************************************************/
void DynamixelX430::GoalPosition(uint8_t id, uint16_t data){
  x430_packetHandler->write4ByteTxOnly(x430_portHandler, id, ADDR_X430_GOAL_POSITION, data);
}

/****************************************************************************************/
/* X430 Synch Write                                                                    */
/****************************************************************************************/
/* This instruction is used to control multiple Dynamixels simultaneously with a single */
/* Instruction Packet transmission. When this instruction is used, several instructions */ 
/* can be transmitted at once, so that the communication time is reduced when multiple  */
/* Dynamixels are connected in a single channel. However, the SYNC WRITE instruction can*/
/* only be used to a single address with an identical length of data over connected     */
/* Dynamixels. ID should be transmitted as Broadcasting ID.                             */
/*                                                                                      */
/* Item           Description                                                           */  
/* ----           -----------                                                           */
/* Instruction    0x83                                                                  */
/* Length         ((L + 1) * N) + 4, L:Data Length, N:Number of Dynamixel               */
/* Parameter 1    Starting address                                                      */
/* Parameter 2    Length of Data to write                                               */
/* Parameter 3    [1st Device] ID                                                       */
/* Parameter 4    [1st Device] 1st Byte                                                 */
/* Parameter 5    [1st Device] 2nd Byte                                                 */
/* …              …                                                                    */
/* Parameter L+3  [1st Device] L-th Byte                                                */
/* Parameter L+4  [2nd Device] ID                                                       */
/* Parameter L+5  [2nd Device] 1st Byte                                                 */
/* Parameter L+6  [2nd Device] 2nd Byte                                                 */
/* …              …                                                                    */
/* Parameter 2L+4 [2nd Device] L-th Byte                                                */    
/****************************************************************************************/

// Need to debug synchWrite. Not working.
void DynamixelX430::SynchWrite(uint16_t start_address, uint16_t data_length, uint8_t *param, uint16_t param_length){
  x430_packetHandler->syncWriteTxOnly(x430_portHandler, start_address, data_length, param, param_length);
}

void DynamixelX430::writeByte(uint8_t id, uint16_t address, uint8_t data){
  x430_packetHandler->write1ByteTxOnly(x430_portHandler, id, address, data);
}

void DynamixelX430::writeWord(uint8_t id, uint16_t address, uint16_t data){
  x430_packetHandler->write2ByteTxOnly(x430_portHandler, id, address, data);
}

//uint8_t DynamixelX430::readByte(uint8_t id, uint8_t address) {
//  uint8_t val;
//  uint8_t dxl_error = 0;                          // Dynamixel error
//  int dxl_comm_result = x430_packetHandler->read1ByteTxRx(x430_portHandler, id, address, &val, &dxl_error);
//  return (dxl_comm_result == COMM_SUCCESS) ? val : 0xff;
//}
//
//uint16_t DynamixelX430::readWord(uint8_t id, uint8_t address) {
//  uint16_t val;
//  uint8_t dxl_error = 0;                          // Dynamixel error
//  int dxl_comm_result = x430_packetHandler->read2ByteTxRx(x430_portHandler, id, address, &val, &dxl_error);
//  return (dxl_comm_result == COMM_SUCCESS) ? val : 0xff;
//}

/****************************************************************************************/
/* End X430 Class                                                                      */
/****************************************************************************************/

/****************************************************************************************/
/* X430 Initializaton                                                                  */
/****************************************************************************************/
DynamixelX430  X430; 

///****************************************************************************************/
///* x430 Initialization Function                                                         */
///****************************************************************************************/
//void initX430(){    
//
//    X430.LED(L11,1);
//    X430.LED(L12,1);
//    X430.LED(L13,1);
//    X430.LED(L21,1);
//    X430.LED(L22,1);
//    X430.LED(L23,1);
//    X430.LED(L31,1);
//    X430.LED(L32,1);
//    X430.LED(L33,1);
//    X430.LED(L41,1);
//    X430.LED(L42,1);
//    X430.LED(L43,1);
//    
//    delay(10);
//    X430.TorqueEnable(L11,1);
//    delay(10);
//    X430.TorqueEnable(L12,1);
//    delay(10);
//    X430.TorqueEnable(L13,1);
//    delay(10);
//    X430.TorqueEnable(L21,1);
//    delay(10);
//    X430.TorqueEnable(L22,1);
//    delay(10);
//    X430.TorqueEnable(L23,1);
//    delay(10);
//    X430.TorqueEnable(L31,1);
//    delay(10);
//    X430.TorqueEnable(L32,1);
//    delay(10);
//    X430.TorqueEnable(L33,1);
//    delay(10);
//    X430.TorqueEnable(L41,1);
//    delay(10);
//    X430.TorqueEnable(L42,1);
//    delay(10);
//    X430.TorqueEnable(L43,1);
//    delay(250);
//
//    X430.LED(L11,0);
//    X430.LED(L12,0);
//    X430.LED(L13,0);
//    X430.LED(L21,0);
//    X430.LED(L22,0);
//    X430.LED(L23,0);
//    X430.LED(L31,0);
//    X430.LED(L32,0);
//    X430.LED(L33,0);
//    X430.LED(L41,0);
//    X430.LED(L42,0);
//    X430.LED(L43,0);
//    delay(100);
//
//    GaitTx();
//    X430.GoalPosition(L11,mechGV.uTheta0[0]);
//    delay(10);
//    X430.GoalPosition(L21,mechGV.uTheta1[0]);
//    delay(10);
//    X430.GoalPosition(L31,mechGV.uTheta2[0]);
//    delay(10);
//    X430.GoalPosition(L41,mechGV.uTheta3[0]);
//    delay(250);
//
//    delay(10);
//    X430.GoalPosition(L12,mechGV.uTheta0[1]);
//    delay(10);
//    X430.GoalPosition(L22,mechGV.uTheta1[1]);
//    delay(10);
//    X430.GoalPosition(L32,mechGV.uTheta2[1]);
//    delay(10);
//    X430.GoalPosition(L42,mechGV.uTheta3[1]);
//    delay(250);
//
//    X430.GoalPosition(L13,mechGV.uTheta0[2]);
//    delay(10);
//    X430.GoalPosition(L23,mechGV.uTheta1[2]);
//    delay(10);
//    delay(10);
//    X430.GoalPosition(L33,mechGV.uTheta2[2]);
//    delay(10);
//    X430.GoalPosition(L43,mechGV.uTheta3[2]);
//    delay(250);
//}
/****************************************************************************************/
/* End Mech Dynamixel X430 Header                                                      */
/****************************************************************************************/
#endif              
