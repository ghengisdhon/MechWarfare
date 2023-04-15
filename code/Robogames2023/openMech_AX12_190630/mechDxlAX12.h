#ifndef mechAX12_h
#define mechAX12_h

/******************************************************* ********************************/
/* AX12 Function Prototyping                                                            */
/****************************************************************************************/
void  initAX12();

/****************************************************************************************/
/* Declare AX12  Class                                                                  */
/****************************************************************************************/
class DynamixelAX12
{
  private:
  dynamixel::PortHandler *ax_portHandler;
  dynamixel::PacketHandler *ax_packetHandler;
  
  public:
  void begin(const char* device_name, float protocol_version, uint32_t baud_rate);
  bool setBaudrate(uint32_t baud_rate);
  void ReturnTimeDelay(uint8_t id, uint8_t data);
  void CWAngleLimit(uint8_t id, uint16_t data);
  void CCWAngleLimit(uint8_t id, uint16_t data);
  void MaxTorque(uint8_t id, uint16_t data);
  void StatusReturnLevel(uint8_t id, uint8_t data);
  void TorqueEnable(uint8_t id, uint8_t data);
  void LED(uint8_t id, uint8_t data);
  void CWComplianceMargin(uint8_t id, uint8_t data);
  void CCWComplianceMargin(uint8_t id, uint8_t data);
  void CWComplianceSlope(uint8_t id, uint8_t data);
  void CCWComplianceSlope(uint8_t id, uint8_t data);
  void GoalPosition(uint8_t id, uint16_t data);
  void MovingSpeed(uint8_t id, uint16_t data);
  void TorqueLimit(uint8_t id, uint16_t data);
  void Punch(uint8_t id, uint16_t data);

  void SynchWrite(uint16_t start_address, uint16_t data_length, uint8_t *param, uint16_t param_length);
//  void writeByte(uint8_t id, uint16_t address, uint8_t data);
//  void writeWord(uint8_t id, uint16_t address, uint16_t data);
//  uint8_t readByte(uint8_t id, uint8_t address);
//  uint16_t readWord(uint8_t id, uint8_t address);

};

/****************************************************************************************/
/* AX12  Begin                                                                          */
/****************************************************************************************/
void DynamixelAX12::begin(const char* device_name, float protocol_version, uint32_t baud_rate ){
  
  // Initialize PortHandler Instance
  // Get methods and members of PortHandlerLinux or PortHandlerWindows
  ax_portHandler = dynamixel::PortHandler::getPortHandler(device_name);

  // Initialize PacketHandler Instance
  // Set the protocol version
  
  // Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
  ax_packetHandler = dynamixel::PacketHandler::getPacketHandler(protocol_version);

  // Open Port
  if (!ax_portHandler->openPort()) {
    SerialUSB.print("Failed to open the Dynamixel AX12 port!\n");
  }

  // Set Port Baudrate
  // Baud Rate determines serial communication speed between a controller and DYNAMIXELs.
  // Value        Baud Rate(bps)  Margin of Error
  // -----        --------------  ---------------
  // 1(Default)   1M              0.000%
  // 3            500,000         0.000%
  // 4            400,000         0.000%
  // 7            250,000         0.000%
  // 9            200,000         0.000%
  // 16           115200         -2.124%
  // 34           57600           0.794%
  // 103          19200          -0.160%
  // 207          9600           -0.160%
  ax_portHandler->setBaudRate(baud_rate);
}
/****************************************************************************************/
/* AX12  Return Time Delay                                                              */
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
void DynamixelAX12::ReturnTimeDelay(uint8_t id, uint8_t data){
  ax_packetHandler->write1ByteTxOnly(ax_portHandler, id, ADDR_AX12_RETURN_TIME_DELAY, data);
}

/****************************************************************************************/
/* AX12  Angle Limits                                                                   */
/****************************************************************************************/
/* The angle limit allows the motion to be restrained. The range and the unit of the    */
/* value is the same as Goal Position(Address 30, 31).                                  */
/*                                                                                      */
/*  CW Angle Limit: the minimum value of Goal Position(Address 30, 31)                  */
/*  CCW Angle Limit: the maximum value of Goal Position(Address 30, 31) The following   */
/*  two modes can be set pursuant to the value of CW and CCW.                           */
/*                                                                                      */ 
/*  Operation Type  CW / CCW                                                            */
/*  --------------  --------                                                            */
/*  Wheel Mode      both are 0                                                          */
/*  Joint Mode      neither are 0                                                       */
/****************************************************************************************/
void DynamixelAX12::CWAngleLimit(uint8_t id, uint16_t data){
  ax_packetHandler->write2ByteTxOnly(ax_portHandler, id, ADDR_AX12_CW_ANGLE_LIMIT, data);
}

void DynamixelAX12::CCWAngleLimit(uint8_t id, uint16_t data){
  ax_packetHandler->write2ByteTxOnly(ax_portHandler, id, ADDR_AX12_CCW_ANGLE_LIMIT, data);
}

/****************************************************************************************/
/* AX12  Max Torque                                                                     */
/****************************************************************************************/
/* It is the torque value of maximum output. 0 to 1,023 (0x3FF) can be used, and the    */
/* unit is about 0.1%. For example, Data 1,023 (0x3FF) means that Dynamixel will use    */
/* 100% of the maximum torque it can produce while Data 512 (0x200) means that Dynamixel*/  
/* will use 50% of the maximum torque. When the power is turned on, Torque Limit        */
/* (Address 34 and 35) uses the value as the initial value.                             */
/****************************************************************************************/
void DynamixelAX12::MaxTorque(uint8_t id, uint16_t data){
  ax_packetHandler->write2ByteTxOnly(ax_portHandler, id, ADDR_AX12_MAX_TORQUE, data);
}

/****************************************************************************************/
/* AX12  Status Return Level                                                            */
/****************************************************************************************/
/* This value decides how to return Status Packet when Dynamixel receives an            */
/* Instruction Packet.                                                                  */
/*                                                                                      */
/* Value  Responding Instructions   Description                                         */
/* -----  -----------------------   -----------                                         */
/* 0      PING Instruction          Status Packet will not be returned for all          */
/*                                  Instructions                                        */
/* 1      PING Instruction          READ Instruction  Status Packet will be returned    */
/*                                  only for READ Instruction                           */
/* 2      All Instructions          Status Packet will be returned for all Instructions */
/****************************************************************************************/
void DynamixelAX12::StatusReturnLevel(uint8_t id, uint8_t data){
  ax_packetHandler->write1ByteTxOnly(ax_portHandler, id, ADDR_AX12_STATUS_RETURN_LEVEL, data);
}

/****************************************************************************************/
/* AX12  Torque Enable                                                                  */
/****************************************************************************************/
/* Value  Description                                                                   */
/* -----  -----------                                                                   */ 
/* 0      Turn off the torque(Free run state)                                           */
/* 1      Turn on the torque and lock EEPROM area                                       */
/****************************************************************************************/
void DynamixelAX12::TorqueEnable(uint8_t id, uint8_t data){
  ax_packetHandler->write1ByteTxOnly(ax_portHandler, id, ADDR_AX12_TORQUE_ENABLE, data);
}

/****************************************************************************************/
/* AX12  LED                                                                            */
/****************************************************************************************/
/* Turn on or turn off the LED on Dynamixel.                                            */
/* Bit          Description                                                             */
/*                                                                                      */
/* ---          -----------                                                             */
/* 0(Default)   Turn OFF the LED                                                        */
/* 1            Turn ON the LED                                                         */
/****************************************************************************************/
void DynamixelAX12::LED(uint8_t id, uint8_t data){
  ax_packetHandler->write1ByteTxOnly(ax_portHandler, id, ADDR_AX12_LED, data);
}

/****************************************************************************************/
/* AX12  Compliance Margins                                                             */
/****************************************************************************************/
/* It exists in each direction of CW/CCW and means the error between goal position and  */
/* present position. The range of the value is 0~255, and the unit is the same as Goal  */
/* Position.(Address 30,31) The greater the value, the more difference occurs.          */
/****************************************************************************************/
void DynamixelAX12::CWComplianceMargin(uint8_t id, uint8_t data){
  ax_packetHandler->write1ByteTxOnly(ax_portHandler, id, ADDR_AX12_CW_COMPLIANCE_MARGIN, data);
}

void DynamixelAX12::CCWComplianceMargin(uint8_t id, uint8_t data){
  ax_packetHandler->write1ByteTxOnly(ax_portHandler, id, ADDR_AX12_CCW_COMPLIANCE_MARGIN, data);
}

/****************************************************************************************/
/* AX12  Compliance Slope                                                               */
/****************************************************************************************/
/* It exists in each direction of CW/CCW and sets the level of Torque near the goal     */
/* position. Compliance Slope is set in 7 steps, the higher the value, the more         */
/* flexibility is obtained. Data representative value is actually used value. That is,  */
/* even if the value is set to 25, 16 is used internally as the representative value.   */
/*                                                                                      */
/* Step Data Value          Data Representative Value                                   */
/* ---- ----------          -------------------------                                   */
/* 1    0(0x00) ~ 3(0x03)   2(0x02)                                                     */
/* 2    4(0x04) ~ 7(0x07)   4(0x04)                                                     */
/* 3    8(0x08)~15(0x0F)    8(0x08)                                                     */
/* 4    16(0x10)~31(0x1F)   16(0x10)                                                    */
/* 5    32(0x20)~63(0x3F)   32(0x20)                                                    */
/* 6    64(0x40)~127(0x7F)  64(0x40)                                                    */
/* 7    128(0x80)~254(0xFE) 128(0x80)                                                   */
/****************************************************************************************/
void DynamixelAX12::CWComplianceSlope(uint8_t id, uint8_t data){
  ax_packetHandler->write1ByteTxOnly(ax_portHandler, id, ADDR_AX12_CW_COMPLIANCE_SLOPE, data);
}

void DynamixelAX12::CCWComplianceSlope(uint8_t id, uint8_t data){
  ax_packetHandler->write1ByteTxOnly(ax_portHandler, id, ADDR_AX12_CCW_COMPLIANCE_SLOPE, data);
}

/****************************************************************************************/
/* AX12  Goal Position                                                                  */
/****************************************************************************************/
/* It is a position value of destination. 0 ~ 1,023 (0x3FF) is available. The unit is   */
/* 0.29°. If Goal Position is out of the range, Angle Limit Error Bit (Bit 1) of Status */ 
/* Packet is returned as ‘1’ and Alarm is triggered as set in Alarm LED/Shutdown.       */
/****************************************************************************************/
void DynamixelAX12::GoalPosition(uint8_t id, uint16_t data){
  ax_packetHandler->write2ByteTxOnly(ax_portHandler, id, ADDR_AX12_GOAL_POSITION, data);
}
/****************************************************************************************/
/* AX12  Moving Speed                                                                   */
/****************************************************************************************/
/* It is a moving speed to Goal Position. The range and the unit of the value may vary  */ 
/* depending on the operation mode.                                                     */
/*                                                                                      */
/* Joint Mode                                                                           */
/* 0 ~ 1,023(0x3FF) can be used, and the unit is about 0.111rpm. If it is set to 0, it  */
/* means the maximum rpm of the motor is used without controlling the speed. If it is   */
/* 1023, it is about 114rpm. For example, if it is set to 300, it is about 33.3 rpm.    */
/*                                                                                      */
/* Wheel Mode                                                                           */
/* 0 ~ 2,047(0x7FF) can be used, the unit is about 0.1%. If a value in the range of     */
/* 0 ~ 1,023 is used, it is stopped by setting to 0 while rotating to CCW direction. If */
/* a value in the range of 1,024 ~ 2,047 is used, it is stopped by setting to 1,024     */
/* while rotating to CW direction. That is, the 10th bit becomes the direction bit to   */
/* control the direction. In Wheel Mode, only the output control is possible, not       */
/* speed. For example, if it is set to 512, it means the output is controlled by 50% of */
/* the maximum output.                                                                  */
/****************************************************************************************/    
void DynamixelAX12::MovingSpeed(uint8_t id, uint16_t data){
  ax_packetHandler->write2ByteTxOnly(ax_portHandler, id, ADDR_AX12_MOVING_SPEED, data);
}

/****************************************************************************************/
/* AX12  Torque Limit                                                                   */
/****************************************************************************************/
/* It is the value of the maximum torque limit. 0 ~ 1,023(0x3FF) is available, and the 
 * unit is about 0.1%. For example, if the value is 512, it is about 50%; that means 
 * only 50% of the maximum torque will be used. If the power is turned on, the value of 
 * Max Torque (Address 14, 15) is used as the initial value                             */
/****************************************************************************************/
void DynamixelAX12::TorqueLimit(uint8_t id, uint16_t data){
  ax_packetHandler->write2ByteTxOnly(ax_portHandler, id, ADDR_AX12_TORQUE_LIMIT, data);
}

/****************************************************************************************/
/* AX12  Punch                                                                          */
/****************************************************************************************/
/* Minimum current to drive motor. This value ranges from 0x20 to 0x3FF.                */
/****************************************************************************************/
void DynamixelAX12::Punch(uint8_t id, uint16_t data){
  ax_packetHandler->write2ByteTxOnly(ax_portHandler, id, ADDR_AX12_PUNCH, data);
}

/****************************************************************************************/
/* AX12  Synch Write                                                                    */
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
/* …              …                                                                   */
/* Parameter 2L+4 [2nd Device] L-th Byte                                                */    
/****************************************************************************************/

// Need to debug synchWrite. Not working.
void DynamixelAX12::SynchWrite(uint16_t start_address, uint16_t data_length, uint8_t *param, uint16_t param_length){
  ax_packetHandler->syncWriteTxOnly(ax_portHandler, start_address, data_length, param, param_length);
}
//
//
//void DynamixelAX12::writeByte(uint8_t id, uint16_t address, uint8_t data){
//  ax_packetHandler->write1ByteTxOnly(ax_portHandler, id, address, data);
//}
//
//void DynamixelAX12::writeWord(uint8_t id, uint16_t address, uint16_t data){
//  ax_packetHandler->write2ByteTxOnly(ax_portHandler, id, address, data);
//}
//
//uint8_t DynamixelAX12::readByte(uint8_t id, uint8_t address) {
//  uint8_t val;
//  uint8_t dxl_error = 0;                          // Dynamixel error
//  int dxl_comm_result = ax_packetHandler->read1ByteTxRx(ax_portHandler, id, address, &val, &dxl_error);
//  return (dxl_comm_result == COMM_SUCCESS) ? val : 0xff;
//}
//
//uint16_t DynamixelAX12::readWord(uint8_t id, uint8_t address) {
//  uint16_t val;
//  uint8_t dxl_error = 0;                          // Dynamixel error
//  int dxl_comm_result = ax_packetHandler->read2ByteTxRx(ax_portHandler, id, address, &val, &dxl_error);
//  return (dxl_comm_result == COMM_SUCCESS) ? val : 0xff;
//}

/****************************************************************************************/
/* End AX12  Class                                                                      */
/****************************************************************************************/

/****************************************************************************************/
/* AX12 Class Declaration                                                               */
/****************************************************************************************/
DynamixelAX12 AX12; 

/****************************************************************************************/
/* AX12 Initialization Function                                                         */
/****************************************************************************************/
void initAX12(){    
//  /**************************************************************************************/  
//  /* Torque Disable                                                                     */
//  /**************************************************************************************/
//  AX12.TorqueEnable(L11,0);
//  AX12.TorqueEnable(L12,0);
//  AX12.TorqueEnable(L13,0);
//  AX12.TorqueEnable(L21,0);
//  AX12.TorqueEnable(L22,0);
//  AX12.TorqueEnable(L23,0);
//  AX12.TorqueEnable(L31,0);
//  AX12.TorqueEnable(L32,0);
//  AX12.TorqueEnable(L33,0);
//  AX12.TorqueEnable(L41,0);
//  AX12.TorqueEnable(L42,0);
//  AX12.TorqueEnable(L43,0);
//  //*********************************************************************************
//
//  //*********************************************************************************
//  // Status Return Level 
//  AX12.StatusReturnLevel(L11,0);
//  AX12.StatusReturnLevel(L12,0);
//  AX12.StatusReturnLevel(L13,0);
//  AX12.StatusReturnLevel(L21,0);
//  AX12.StatusReturnLevel(L22,0);
//  AX12.StatusReturnLevel(L23,0);
//  AX12.StatusReturnLevel(L31,0);
//  AX12.StatusReturnLevel(L32,0);
//  AX12.StatusReturnLevel(L33,0);
//  AX12.StatusReturnLevel(L41,0);
//  AX12.StatusReturnLevel(L42,0);
//  AX12.StatusReturnLevel(L43,0);
//  //*********************************************************************************
//  // Return Delay Time
//  // It is the delay time per data value that takes from the transmission of Instruction Packet 
//  // until the return of Status Packet. 0 to 254 (0xFE) can be used, and the delay time per data 
//  // value is 2 usec. That is to say, if the data value is 10, 20 usec is delayed. The initial 
//  // value is 250 (0xFA) (i.e., 0.5 msec).
//  //*********************************************************************************
//  // Status Return Level 
//  AX12.ReturnTimeDelay(L11,0);
//  AX12.ReturnTimeDelay(L12,0);
//  AX12.ReturnTimeDelay(L13,0);
//  AX12.ReturnTimeDelay(L21,0);
//  AX12.ReturnTimeDelay(L22,0);
//  AX12.ReturnTimeDelay(L23,0);
//  AX12.ReturnTimeDelay(L31,0);
//  AX12.ReturnTimeDelay(L32,0);
//  AX12.ReturnTimeDelay(L33,0);
//  AX12.ReturnTimeDelay(L41,0);
//  AX12.ReturnTimeDelay(L42,0);
//  AX12.ReturnTimeDelay(L43,0);  
//  //*********************************************************************************
//  // CW/CCW Angle Limit
//  // The angle limit allows the motion to be restrained.
//  // The range and the unit of the value is the same as Goal Position(Address 30, 31).
//  //*********************************************************************************
//  // Joint 1 CW angle Limit
//  AX12.CWAngleLimit(L11, Joint1CWlimF);
//  AX12.CWAngleLimit(L21, Joint1CWlimR); 
//  AX12.CWAngleLimit(L31, Joint1CWlimR);   
//  AX12.CWAngleLimit(L41, Joint1CWlimF);
//  // Joint 1 CCW angle Limit
//  AX12.CCWAngleLimit(L11, Joint1CCWlimF);  
//  AX12.CCWAngleLimit(L21, Joint1CCWlimR);   
//  AX12.CCWAngleLimit(L31, Joint1CCWlimR);
//  AX12.CCWAngleLimit(L41, Joint1CCWlimF); 
//  // Joint 2 CW Limit
//  AX12.CWAngleLimit(L12, Joint2CWlimF);
//  AX12.CWAngleLimit(L22, Joint2CWlimR);  
//  AX12.CWAngleLimit(L32, Joint2CWlimR);
//  AX12.CWAngleLimit(L42, Joint2CWlimF);
//  // Joint 2 CCW Limit
//  AX12.CCWAngleLimit(L12, Joint2CCWlimF);  
//  AX12.CCWAngleLimit(L22, Joint2CCWlimR); 
//  AX12.CCWAngleLimit(L32, Joint2CCWlimR);
//  AX12.CCWAngleLimit(L42, Joint2CCWlimF); 
//  // Joint 3 CW Limit  
//  AX12.CWAngleLimit(L13, Joint3CWlimF);
//  AX12.CWAngleLimit(L23, Joint3CWlimR);
//  AX12.CWAngleLimit(L33, Joint3CWlimR);
//  AX12.CWAngleLimit(L43, Joint3CWlimF);
//  // Joint 3 CCW Limit
//  AX12.CCWAngleLimit(L13, Joint3CCWlimF);
//  AX12.CCWAngleLimit(L23, Joint3CCWlimR);    
//  AX12.CCWAngleLimit(L33, Joint3CCWlimR);    
//  AX12.CCWAngleLimit(L43, Joint3CCWlimF);  
//
//  //********************************************************************************* 
//  // Max Torque (EEPROM Setting)
//  // It is the torque value of maximum output. 0 to 1023 (0x3FF) can be used, and the unit is about 0.1%.
//  // For example, Data 1023 (0x3FF) means that Dynamixel will use 100% of the maximum torque it can 
//  // produce while Data 512 (0x200) means that Dynamixel will use 50% of the maximum torque. When the 
//  // power is turned on, Torque Limit (Addresses 34 and 35) uses the value as the initial value.
//  //*********************************************************************************  
//  uint16_t MaxTorque = 1023;  
//  AX12.MaxTorque(L11,MaxTorque); 
//  AX12.MaxTorque(L12,MaxTorque);
//  AX12.MaxTorque(L13,MaxTorque);
//  AX12.MaxTorque(L21,MaxTorque);
//  AX12.MaxTorque(L22,MaxTorque);
//  AX12.MaxTorque(L23,MaxTorque);
//  AX12.MaxTorque(L31,MaxTorque);
//  AX12.MaxTorque(L32,MaxTorque);
//  AX12.MaxTorque(L33,MaxTorque);
//  AX12.MaxTorque(L41,MaxTorque);
//  AX12.MaxTorque(L42,MaxTorque);
//  AX12.MaxTorque(L43,MaxTorque);
//
//  //*********************************************************************************
//  // Compliance Margin
//  // It exists in each direction of CW/CCW and means the error between goal position and present position.
//  // The range of the value is 0~255, and the unit is the same as Goal Position.(Address 30,31)
//  // The greater the value, the more difference occurs.
//  //*********************************************************************************  
//  byte CW_CMargin  = 1;
//  byte CCW_CMargin = 1;
//
//  // CW Compliance Margin
//  AX12.CWComplianceMargin(L11,CW_CMargin);
//  AX12.CWComplianceMargin(L12,CW_CMargin);
//  AX12.CWComplianceMargin(L13,CW_CMargin);
//  AX12.CWComplianceMargin(L21,CW_CMargin);
//  AX12.CWComplianceMargin(L22,CW_CMargin);
//  AX12.CWComplianceMargin(L23,CW_CMargin);
//  AX12.CWComplianceMargin(L31,CW_CMargin);
//  AX12.CWComplianceMargin(L32,CW_CMargin);
//  AX12.CWComplianceMargin(L33,CW_CMargin);
//  AX12.CWComplianceMargin(L41,CW_CMargin);
//  AX12.CWComplianceMargin(L42,CW_CMargin);
//  AX12.CWComplianceMargin(L43,CW_CMargin);
//
//  // CCW Compliance Margin 
//  AX12.CCWComplianceMargin(L11,CCW_CMargin);
//  AX12.CCWComplianceMargin(L12,CCW_CMargin);
//  AX12.CCWComplianceMargin(L13,CCW_CMargin);
//  AX12.CCWComplianceMargin(L21,CCW_CMargin);
//  AX12.CCWComplianceMargin(L22,CCW_CMargin);
//  AX12.CCWComplianceMargin(L23,CCW_CMargin);
//  AX12.CCWComplianceMargin(L31,CCW_CMargin);
//  AX12.CCWComplianceMargin(L32,CCW_CMargin);
//  AX12.CCWComplianceMargin(L33,CCW_CMargin);
//  AX12.CCWComplianceMargin(L41,CCW_CMargin);
//  AX12.CCWComplianceMargin(L42,CCW_CMargin);
//  AX12.CCWComplianceMargin(L43,CCW_CMargin);
//
//  //*********************************************************************************
//  // Compliance Slope
//  // It exists in each direction of CW/CCW and sets the level of Torque near the goal position.
//  // Compliance Slope is set in 7 steps, the higher the value, the more flexibility is obtained.
//  // Data representative value is actually used value.  That is, even if the value is set to 25, 
//  // 16 is used internally as the representative value.
//  //*********************************************************************************  
//  byte CW_CSLOPE  = 32;
//  byte CCW_CSLOPE = 32;
//
//  // CW
//  AX12.CWComplianceSlope(L11,CW_CSLOPE);
//  AX12.CWComplianceSlope(L12,CW_CSLOPE);
//  AX12.CWComplianceSlope(L13,CW_CSLOPE);
//  AX12.CWComplianceSlope(L21,CW_CSLOPE);
//  AX12.CWComplianceSlope(L22,CW_CSLOPE);
//  AX12.CWComplianceSlope(L23,CW_CSLOPE);
//  AX12.CWComplianceSlope(L31,CW_CSLOPE);
//  AX12.CWComplianceSlope(L32,CW_CSLOPE);
//  AX12.CWComplianceSlope(L33,CW_CSLOPE);
//  AX12.CWComplianceSlope(L41,CW_CSLOPE);
//  AX12.CWComplianceSlope(L42,CW_CSLOPE);
//  AX12.CWComplianceSlope(L43,CW_CSLOPE);
//
//  // CCW
//  AX12.CCWComplianceSlope(L11,CCW_CSLOPE);
//  AX12.CCWComplianceSlope(L12,CCW_CSLOPE);
//  AX12.CCWComplianceSlope(L13,CCW_CSLOPE);
//  AX12.CCWComplianceSlope(L21,CCW_CSLOPE);
//  AX12.CCWComplianceSlope(L22,CCW_CSLOPE);
//  AX12.CCWComplianceSlope(L23,CCW_CSLOPE);
//  AX12.CCWComplianceSlope(L31,CCW_CSLOPE);
//  AX12.CCWComplianceSlope(L32,CCW_CSLOPE);
//  AX12.CCWComplianceSlope(L33,CCW_CSLOPE);
//  AX12.CCWComplianceSlope(L41,CCW_CSLOPE);
//  AX12.CCWComplianceSlope(L42,CCW_CSLOPE);
//  AX12.CCWComplianceSlope(L43,CCW_CSLOPE);  
//
//  //********************************************************************************* 
//  // Moving Speed
//  // It is a moving speed to Goal Position.
//  // The range and the unit of the value may vary depending on the operation mode.
//  //
//  // Join Mode
//  // 0~1023 (0X3FF) can be used, and the unit is about 0.111rpm.
//  // If it is set to 0, it means the maximum rpm of the motor is used without controlling the speed.
//  // If it is 1023, it is about 114rpm.
//  //
//  // For example, if it is set to 300, it is about 33.3 rpm.
//  //*********************************************************************************  
//  uint16_t Joint1_MS  = 511;
//  uint16_t Joint2_MS  = 511;
//  uint16_t Joint3_MS  = 511;
//
//  AX12.MovingSpeed(L11,Joint1_MS);
//  AX12.MovingSpeed(L12,Joint2_MS);
//  AX12.MovingSpeed(L13,Joint3_MS);
//  AX12.MovingSpeed(L21,Joint1_MS);
//  AX12.MovingSpeed(L22,Joint2_MS);
//  AX12.MovingSpeed(L23,Joint3_MS);
//  AX12.MovingSpeed(L31,Joint1_MS);
//  AX12.MovingSpeed(L32,Joint2_MS);
//  AX12.MovingSpeed(L33,Joint3_MS);
//  AX12.MovingSpeed(L41,Joint1_MS);
//  AX12.MovingSpeed(L42,Joint2_MS);
//  AX12.MovingSpeed(L43,Joint3_MS);
//
//  //*********************************************************************************
//  // Torque Limit
//  // It is the value of the maximum torque limit. 0 to 1023 (0x3FF) is available, and the unit is about 0.1%.  
//  //
//  // For example, if the value is 512, it is about 50%; that means only 50% of the maximum torque will be used.
//  // If the power is turned on, the value of Max Torque is used as the initial value.
//  //
//  // Notes: If the function of Alarm Shutdown is triggered, the motor loses its torque because the value 
//  // becomes 0.  Once error conditions are resolved and this value is changed to the value other than 0, 
//  // the motor can be operated again.
//  //********************************************************************************* 
//  uint16_t TorqueLimit  = 1023;
//
//  AX12.TorqueLimit(L11,TorqueLimit);
//  AX12.TorqueLimit(L12,TorqueLimit);
//  AX12.TorqueLimit(L13,TorqueLimit);
//  AX12.TorqueLimit(L21,TorqueLimit);
//  AX12.TorqueLimit(L22,TorqueLimit);
//  AX12.TorqueLimit(L23,TorqueLimit);
//  AX12.TorqueLimit(L31,TorqueLimit);
//  AX12.TorqueLimit(L32,TorqueLimit);
//  AX12.TorqueLimit(L33,TorqueLimit);
//  AX12.TorqueLimit(L41,TorqueLimit);
//  AX12.TorqueLimit(L42,TorqueLimit);
//  AX12.TorqueLimit(L43,TorqueLimit);
//
//  //*********************************************************************************   
//  // Punch
//  // Current to drive motor is at minimum.
//  // Can choose vales from 0x20 to 0x3FF.
//  //*********************************************************************************  
//  uint16_t Punch  = 32;
//  
//  AX12.Punch(L11,Punch);
//  AX12.Punch(L12,Punch);
//  AX12.Punch(L13,Punch);
//  AX12.Punch(L21,Punch);
//  AX12.Punch(L22,Punch);
//  AX12.Punch(L23,Punch);
//  AX12.Punch(L31,Punch);
//  AX12.Punch(L32,Punch);
//  AX12.Punch(L33,Punch);
//  AX12.Punch(L41,Punch);
//  AX12.Punch(L42,Punch);
//  AX12.Punch(L43,Punch);
//
//  //*********************************************************************************  
//  // Torque Enable
//  //*********************************************************************************  
//  AX12.TorqueEnable(L11,1);
//  AX12.TorqueEnable(L12,1);
//  AX12.TorqueEnable(L13,1);
//  AX12.TorqueEnable(L21,1);
//  AX12.TorqueEnable(L22,1);
//  AX12.TorqueEnable(L23,1);
//  AX12.TorqueEnable(L31,1);
//  AX12.TorqueEnable(L32,1);
//  AX12.TorqueEnable(L33,1);
//  AX12.TorqueEnable(L41,1);
//  AX12.TorqueEnable(L42,1);
//  AX12.TorqueEnable(L43,1);
}
/****************************************************************************************/
/* End Mech Dynamixel AX12 Header                                                       */
/****************************************************************************************/
#endif 
