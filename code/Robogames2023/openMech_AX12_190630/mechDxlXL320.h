
#ifndef mechXL320_h
#define mechXL320_h
/****************************************************************************************/
/* XL320 Function Prototyping                                                           */
/****************************************************************************************/

/****************************************************************************************/
/* Declare XL320 (Protocol 2) Class                                                     */
/****************************************************************************************/
class DynamixelXL320
{
  private:
  dynamixel::PortHandler *xl32_portHandler;
  dynamixel::PacketHandler *xl320_packetHandler;
  
  public:
  void begin(const char* device_name, float protocol_version, uint32_t baud_rate);
  bool setBaudrate(uint32_t baud_rate);
  void ReturnTimeDelay(uint8_t id, uint8_t data);
  void CWAngleLimit(uint8_t id, uint16_t data);
  void CCWAngleLimit(uint8_t id, uint16_t data);
  void ControlMode(uint8_t id, uint8_t data);
  void MaxTorque(uint8_t id, uint16_t data);
  void StatusReturnLevel(uint8_t id, uint8_t data);
  void TorqueEnable(uint8_t id, uint8_t data);
  void LED(uint8_t id, uint8_t data);
  void DGain(uint8_t id, uint8_t data);
  void IGain(uint8_t id, uint8_t data);
  void PGain(uint8_t id, uint8_t data);
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
/* XL320 Begin                                                                          */
/****************************************************************************************/
void DynamixelXL320::begin(const char* device_name, float protocol_version, uint32_t baud_rate ){
  
  // Initialize PortHandler Instance
  // Get methods and members of PortHandlerLinux or PortHandlerWindows
  xl32_portHandler = dynamixel::PortHandler::getPortHandler(device_name);

  // Initialize PacketHandler Instance
  // Set the protocol version
  
  // Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
  xl320_packetHandler = dynamixel::PacketHandler::getPacketHandler(protocol_version);

  // Open Port
  if (!xl32_portHandler->openPort()) {
    SerialUSB.print("Failed to open the Dynamixel XL320 port!\n");
  }

/* Baud Rate determines serial communication speed between a controller and DYNAMIXELs.
/* Value  Baud Rate
/* -----  ---------                                                                     */
/* 0      9,600 bps                                                                     */
/* 1      57,600 bps                                                                    */
/* 2      115,200 bps                                                                   */
/* 3      1 Mbps                                                                        */
  xl32_portHandler->setBaudRate(baud_rate);
}
/****************************************************************************************/
/* XL320 Return Time Delay                                                              */
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
void DynamixelXL320::ReturnTimeDelay(uint8_t id, uint8_t data){
  xl320_packetHandler->write1ByteTxOnly(xl32_portHandler, id, ADDR_XL320_RETURN_TIME_DELAY, data);
}

/****************************************************************************************/
/* XL320 Angle Limits                                                                   */
/****************************************************************************************/
/* The angle limit allows the motion to be restrained. The range and the unit of the    */
/* value is the same as Goal Position(Address 30, 31).                                  */
/****************************************************************************************/
void DynamixelXL320::CWAngleLimit(uint8_t id, uint16_t data){
  xl320_packetHandler->write2ByteTxOnly(xl32_portHandler, id, ADDR_XL320_CW_ANGLE_LIMIT, data);
}

void DynamixelXL320::CCWAngleLimit(uint8_t id, uint16_t data){
  xl320_packetHandler->write2ByteTxOnly(xl32_portHandler, id, ADDR_XL320_CCW_ANGLE_LIMIT, data);
}
/****************************************************************************************/
/* XL320 Control Mode                                                   */
/****************************************************************************************/
/* Value  Mode                                                                          */
/* -----  ----                                                                          */
/* 1      Wheel Mode                                                                    */
/* 2      Joint Mode                                                                    */
/****************************************************************************************/
void DynamixelXL320::ControlMode(uint8_t id, uint8_t data){
  xl320_packetHandler->write1ByteTxOnly(xl32_portHandler, id, ADDR_XL320_CONTROL_MODE, data);
}

/****************************************************************************************/
/* XL320 Max Torque                                                                     */
/****************************************************************************************/
/* It is the torque value of maximum output. 0 to 1,023 (0x3FF) can be used, and the    */
/* unit is about 0.1%. For example, Data 1,023 (0x3FF) means that Dynamixel will use    */
/* 100% of the maximum torque it can produce while Data 512 (0x200) means that          */
/* Dynamixel will use 50% of the maximum torque. When the power is turned on, Torque    */
/*  Limit (Address 34 and 35) uses the value as the initial value.                      */
/****************************************************************************************/
void DynamixelXL320::MaxTorque(uint8_t id, uint16_t data){
  xl320_packetHandler->write2ByteTxOnly(xl32_portHandler, id, ADDR_XL320_MAX_TORQUE, data);
}

/****************************************************************************************/
/* XL320 Status Return Level                                                            */
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
void DynamixelXL320::StatusReturnLevel(uint8_t id, uint8_t data){
  xl320_packetHandler->write1ByteTxOnly(xl32_portHandler, id, ADDR_XL320_STATUS_RETURN_LEVEL, data);
}

/****************************************************************************************/
/* XL320 Torque Enable                                                                  */
/****************************************************************************************/
/* Value  Description                                                                   */
/* -----  -----------                                                                   */ 
/* 0      Turn off the torque(Free run state)                                           */
/* 1      Turn on the torque and lock EEPROM area                                       */
/****************************************************************************************/
void DynamixelXL320::TorqueEnable(uint8_t id, uint8_t data){
  xl320_packetHandler->write1ByteTxOnly(xl32_portHandler, id, ADDR_XL320_TORQUE_ENABLE, data);
}

/****************************************************************************************/
/* XL320 LED                                                                            */
/****************************************************************************************/
/* The combination of bit changes the output color of XL-320.                           */
/* Bit          Output Color                                                            */
/* ---          ------------                                                            */
/* 0            Red                                                                     */
/* 1            Green                                                                   */
/* 2            Blue                                                                    */
/* 0 + 1        Yellow                                                                  */
/* 1 + 2        Cyan                                                                    */
/* 0 + 2        Purple                                                                  */
/* 0 + 1 + 2    White                                                                   */
/****************************************************************************************/
void DynamixelXL320::LED(uint8_t id, uint8_t data){
  xl320_packetHandler->write1ByteTxOnly(xl32_portHandler, id, ADDR_XL320_LED, data);
}

/****************************************************************************************/
/* XL320 PID Gains                                                                      */
/****************************************************************************************/
/* MX series will use the PID controller as a main control method.                      */ 
/*    P gain : value of proportional band.                                              */
/*    I gain : value of integral action.                                                */
/*    D Gain : value of derivative action. Gains values are in between 0 ~ 254.         */
/*                                                                                      */
/*    K<sub>p</sub> = P Gain / 8                                                        */
/*    K<sub>i</sub> = I Gain * 1,000 / 2,048                                            */
/*    K<sub>d</sub> = D Gain * 4 / 1,000                                                */
/*                                                                                      */
/* The relationship between Compliance Slop and PID                                     */
/* Slope  P Gain                                                                        */
/* -----  ------                                                                        */
/* 8      128                                                                           */
/* 16     64                                                                            */
/* 32     32                                                                            */
/* 64     16                                                                            */
/* 128    8                                                                             */
/*                                                                                      */
/* The less the P gain, The larger the back lash, and the weaker the amount of output   */
/* near goal position. At some extent, it is like a combined concept of margine and     */
/* slope. It does not exactly match the previous concept of compliance. So it is        */
/*  obvious if you see the difference in terms of motion. Explanation for PID required. */
/* For the brief explanation about general PID, please refer to the website(link) below.*/
/* http://en.wikipedia.org/wiki/PID_controller                                          */
/* FYI, PID control theory is not only limited to the control of motor(actuator) but is */
/*  a generic theory that can be applied to all kinds of control.                       */
/****************************************************************************************/
void DynamixelXL320::DGain(uint8_t id, uint8_t data){

  xl320_packetHandler->write1ByteTxOnly(xl32_portHandler, id, ADDR_XL320_D_GAIN, data);
}

void DynamixelXL320::IGain(uint8_t id, uint8_t data){
  xl320_packetHandler->write1ByteTxOnly(xl32_portHandler, id, ADDR_XL320_I_GAIN, data);
}

void DynamixelXL320::PGain(uint8_t id, uint8_t data){
  xl320_packetHandler->write1ByteTxOnly(xl32_portHandler, id, ADDR_XL320_P_GAIN, data);
}

/****************************************************************************************/
/* XL320 Goal Position                                                                  */
/****************************************************************************************/
/* It is a position value of destination. 0 ~ 1,023 (0x3FF) is available. The unit is   */
/* 0.29°. If Goal Position is out of the range, Angle Limit Error Bit (Bit 1) of Status */ 
/* Packet is returned as ‘1’ and Alarm is triggered as set in Alarm LED/Shutdown.       */
/****************************************************************************************/
void DynamixelXL320::GoalPosition(uint8_t id, uint16_t data){
  xl320_packetHandler->write2ByteTxOnly(xl32_portHandler, id, ADDR_XL320_GOAL_POSITION, data);
}
/****************************************************************************************/
/* XL320 Moving Speed                                                                   */
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
void DynamixelXL320::MovingSpeed(uint8_t id, uint16_t data){
  xl320_packetHandler->write2ByteTxOnly(xl32_portHandler, id, ADDR_XL320_MOVING_SPEED, data);
}

/****************************************************************************************/
/* XL320 Torque Limit                                                                   */
/****************************************************************************************/
/* It is the value of the maximum torque limit. 0 ~ 1,023(0x3FF) is available, and the  */
/* unit is about 0.1%. For example, if the value is 512, it is about 50%; that means    */
/* only 50% of the maximum torque will be used. If the power is turned on, the value of */
/* Max Torque (Address 14, 15) is used as the initial value                             */
/****************************************************************************************/
void DynamixelXL320::TorqueLimit(uint8_t id, uint16_t data){
  xl320_packetHandler->write2ByteTxOnly(xl32_portHandler, id, ADDR_XL320_TORQUE_LIMIT, data);
}

/****************************************************************************************/
/* XL320 Punch                                                                          */
/****************************************************************************************/
/* Minimum current to drive motor. This value ranges from 0x20 to 0x3FF.                */
/****************************************************************************************/
void DynamixelXL320::Punch(uint8_t id, uint16_t data){
  xl320_packetHandler->write2ByteTxOnly(xl32_portHandler, id, ADDR_XL320_PUNCH, data);
}

/****************************************************************************************/
/* XL320 Synch Write                                                                    */
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
void DynamixelXL320::SynchWrite(uint16_t start_address, uint16_t data_length, uint8_t *param, uint16_t param_length){
  xl320_packetHandler->syncWriteTxOnly(xl32_portHandler, start_address, data_length, param, param_length);
}

//void DynamixelXL320::writeByte(uint8_t id, uint16_t address, uint8_t data){
//  xl320_packetHandler->write1ByteTxOnly(xl32_portHandler, id, address, data);
//}
//
//void DynamixelXL320::writeWord(uint8_t id, uint16_t address, uint16_t data){
//  xl320_packetHandler->write2ByteTxOnly(xl32_portHandler, id, address, data);
//}
//
//uint8_t DynamixelXL320::readByte(uint8_t id, uint8_t address) {
//  uint8_t val;
//  uint8_t dxl_error = 0;                          // Dynamixel error
//  int dxl_comm_result = xl320_packetHandler->read1ByteTxRx(xl32_portHandler, id, address, &val, &dxl_error);
//  return (dxl_comm_result == COMM_SUCCESS) ? val : 0xff;
//}
//
//uint16_t DynamixelXL320::readWord(uint8_t id, uint8_t address) {
//  uint16_t val;
//  uint8_t dxl_error = 0;                          // Dynamixel error
//  int dxl_comm_result = xl320_packetHandler->read2ByteTxRx(xl32_portHandler, id, address, &val, &dxl_error);
//  return (dxl_comm_result == COMM_SUCCESS) ? val : 0xff;
//}

/****************************************************************************************/
/* End XL320 Class                                                                      */
/****************************************************************************************/

/****************************************************************************************/
/* XL320 Initializaton                                                                  */
/****************************************************************************************/
DynamixelXL320  XL320; 

/****************************************************************************************/
/* End Mech Dynamixel XL320 Header                                                      */
/****************************************************************************************/
#endif              
