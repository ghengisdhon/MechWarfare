#ifndef mechDefine_h
#define mechDefine_h

/****************************************************************************************/
/* Mathematical Constants                                                               */
/****************************************************************************************/
#define TwoPI                           (6.28318530717959)      // 2*Pi
#define HalfPI                          (1.5707963267949)       // Pi/2
#define SqrtTwo                         (1.4142135623731)       // sqrt(2)
#define SqrtThree                       (1.73205080756888)      // sqrt(3)
#define InvSqrtTwo                      (0.707106781186547)     // 1/sqrt(2)
#define InvSqrtThree                    (0.577350269189626)     // 1/sqrt(3)
/****************************************************************************************/
/* Conversions                                                                          */
/****************************************************************************************/
#define R2D                             (57.2957795130823)      // Radians to Degress
#define D2R                             (0.0174532925199433)    // Degress to Radians

#define C2R_AX12                        (0.00613592315154256)   // Counts (1023) to Radians
#define R2C_AX12                        (162.974661726101)      // Radians to Counts (1023)
#define Two2C_AX12                      (512)                   // 2/1024 
#define C2Two_AX12                      (0.001953125)           // 1024/2 
#define D2C_AX12                        (3.41F)                 // Deg to Counts 1023/300 (for AX12)
#define C2D_AX12                        (0.293255131964809)     // Counts to Degree 300/1023 (for AX12)

#define C2R                             (0.00613592315154256)   // Counts (1023) to Radians
#define R2C                             (162.974661726101)      // Radians to Counts (1023)
#define Two2C                           (512)                   // 2/1024 
#define C2Two                           (0.001953125)           // 1024/2 
#define D2C                             (3.41F)                 // Deg to Counts 1023/300 (for AX12)
#define C2D                             (0.293255131964809)     // Counts to Degree 300/1023 (for AX12)

//#define C2R                             (0.001534355386369)     // Counts (4095) to Radians
//#define R2C                             (651.7394919613114)     // Radians to Counts (1023)
//#define Two2C                           (2048)                  // 4096/2
//#define C2Two                           (4.882812500000000e-04) // 2/4096
//#define D2C                             (11.375000000000000F)   // Deg to Counts 4095/360 (for X430)
//#define C2D                             (0.087912087912088)     // Counts to Degree 360/4095 (for AX12)
/****************************************************************************************/
/* Leg Definitions                                                                      */
/****************************************************************************************/
#define L11                             (11)              // Leg 1, Joint 1 Dynamixel ID 
#define L12                             (12)              // Leg 1, Joint 2 Dynamixel ID
#define L13                             (13)              // Leg 1, Joint 3 Dynamixel ID
#define L21                             (21)              // Leg 2, Joint 1 Dynamixel ID
#define L22                             (22)              // Leg 2, Joint 2 Dynamixel ID
#define L23                             (23)              // Leg 2, Joint 3 Dynamixel ID
#define L31                             (31)              // Leg 3, Joint 1 Dynamixel ID
#define L32                             (32)              // Leg 3, Joint 2 Dynamixel ID
#define L33                             (33)              // Leg 3, Joint 3 Dynamixel ID
#define L41                             (41)              // Leg 4, Joint 1 Dynamixel ID
#define L42                             (42)              // Leg 4, Joint 2 Dynamixel ID
#define L43                             (43)              // Leg 4, Joint 3 Dynamixel ID
/****************************************************************************************/
/* ISR Timer Defintitions                                                               */
/*                                                                                      */
/*  Timer Interrupt Period (usec)     Freq = 1/(TimerMS1*1e-6)                          */
/*  50000 = 20 hz                                                                       */
/*  40000 = 25 Hz                                                                       */
/*  30000 = 33 Hz                                                                       */
/*  25000 = 40 Hz                                                                       */
/*  20000 = 50 Hz                                                                       */
/*  15000 = 67 Hz                                                                       */
/****************************************************************************************/
#define TimerNumber                       (3)               // ISR Timer 

#define TimerMS1 30000                         // 25 Hz
#define TimerMS2 25000                         // 33 Hz
#define TimerMS3 20000                         // 50 Hz
#define TimerMS4 20000                         // 66 Hz

#define uStepSize1 1.5
#define uStepSize2 2.0
#define uStepSize3 3.0
#define uStepSize4 3.5

#define StepHeight1 (0.375F)
#define StepHeight2 (0.25F)
#define StepHeight3 (0.25F)
#define StepHeight4 (0.25F)

/****************************************************************************************/
/* Define Math Functions to Use                                                         */
/****************************************************************************************/
#define mechCos cos
#define mechSin sin
#define mechAtan2 atan2
#define mechAsin asin
#define mechAcos acos
#define mechSqrt sqrt

//#define mechCos cosLUT
//#define mechSin sinLUT
//#define mechAtan2 atan2Approx
//#define mechAsin asinLUT
//#define mechAcos acosLUT
//#define mechSqrt sqrtNewton // Causes Issues

/****************************************************************************************/
/* OpenCM9.04 Serial Bus                                                                */                                   
/****************************************************************************************/
#define baudXbee                        (57600)           // XBee Baud Rate
#define baudSerial3                     (9600)            // Not USed
#define DXL_BUS_SERIAL1                 ("1")             // Serial1(USART1)  <-OpenCM9.04
#define DXL_BUS_BAUDRATE                (1000000)         // Dynamixel Bus Baud Rate
/****************************************************************************************/
/* AX12 Control Table                                                                   */                                   
/****************************************************************************************/
#define AX12_PROTOCOL_VERSION            (1.0)
#define ADDR_AX12_MODEL_NUMBER           (0)
#define ADDR_AX12_FIRMWARE_VERSION       (2)
#define ADDR_AX12_ID                     (3)
#define ADDR_AX12_BAUDRATE               (4)
#define ADDR_AX12_RETURN_TIME_DELAY      (5)
#define ADDR_AX12_CW_ANGLE_LIMIT         (6)
#define ADDR_AX12_CCW_ANGLE_LIMIT        (8)
#define ADDR_AX12_TEMPERATURE_LIMIT      (11)
#define ADDR_AX12_MIN_VOLTAGE_LIMIT      (12)
#define ADDR_AX12_MAX_VOLTAGE_LIMIT      (13)
#define ADDR_AX12_MAX_TORQUE             (14)
#define ADDR_AX12_STATUS_RETURN_LEVEL    (16)
#define ADDR_AX12_ALARM_LED              (17)
#define ADDR_AX12_SHUTDOWN               (18)
#define ADDR_AX12_TORQUE_ENABLE          (24)
#define ADDR_AX12_LED                    (25)
#define ADDR_AX12_CW_COMPLIANCE_MARGIN   (26)
#define ADDR_AX12_CCW_COMPLIANCE_MARGIN  (27)
#define ADDR_AX12_CW_COMPLIANCE_SLOPE    (28)
#define ADDR_AX12_CCW_COMPLIANCE_SLOPE   (29)
#define ADDR_AX12_GOAL_POSITION          (30)
#define ADDR_AX12_MOVING_SPEED           (32)
#define ADDR_AX12_TORQUE_LIMIT           (34)
#define ADDR_AX12_PRESENT_POSITION       (36)
#define ADDR_AX12_PRESENT_SPEED          (38)
#define ADDR_AX12_PRESENT_LOAD           (40)
#define ADDR_AX12_PRESENT_VOLTAGE        (42)
#define ADDR_AX12_PRESENT_TEMPERATURE    (43)
#define ADDR_AX12_REGISTERED             (44)
#define ADDR_AX12_MOVING                 (46)
#define ADDR_AX12_LOCK                   (47)
#define ADDR_AX12_PUNCH                  (48)
/****************************************************************************************/
/* XL320 Control Table                                                                  */                                   
/****************************************************************************************/
#define XL320_PROTOCOL_VERSION            (2.0)
#define ADDR_XL320_MODEL_NUMBER           (0)
#define ADDR_XL320_FIRMWARE_VERSION       (2)
#define ADDR_XL320_ID                     (3)
#define ADDR_XL320_BAUDRATE               (4)
#define ADDR_XL320_RETURN_TIME_DELAY      (5)
#define ADDR_XL320_CW_ANGLE_LIMIT         (6)
#define ADDR_XL320_CCW_ANGLE_LIMIT        (8)
#define ADDR_XL320_CONTROL_MODE           (11)
#define ADDR_XL320_TEMPERATURE_LIMIT      (12)
#define ADDR_XL320_MIN_VOLTAGE_LIMIT      (13)
#define ADDR_XL320_MAX_VOLTAGE_LIMIT      (14)
#define ADDR_XL320_MAX_TORQUE             (15)
#define ADDR_XL320_STATUS_RETURN_LEVEL    (17)
#define ADDR_XL320_SHUTDOWN               (18)
#define ADDR_XL320_TORQUE_ENABLE          (24)
#define ADDR_XL320_LED                    (25)
#define ADDR_XL320_D_GAIN                 (27)
#define ADDR_XL320_I_GAIN                 (28)
#define ADDR_XL320_P_GAIN                 (29)
#define ADDR_XL320_GOAL_POSITION          (30)
#define ADDR_XL320_MOVING_SPEED           (32)
#define ADDR_XL320_TORQUE_LIMIT           (35)
#define ADDR_XL320_PRESENT_POSITION       (37)
#define ADDR_XL320_PRESENT_SPEED          (39)
#define ADDR_XL320_PRESENT_LOAD           (41)
#define ADDR_XL320_PRESENT_VOLTAGE        (45)
#define ADDR_XL320_PRESENT_TEMPERATURE    (46)
#define ADDR_XL320_REGISTERED             (47)
#define ADDR_XL320_MOVING                 (49)
#define ADDR_XL320_HARDWARE_ERROR_STAUS   (50)
#define ADDR_XL320_PUNCH                  (51)
/****************************************************************************************/
/* X430 Control Table                                                                  */                                   
/****************************************************************************************/
#define X430_PROTOCOL_VERSION             (1.0)
#define ADDR_X430_MODEL_NUMBER            (0)
#define ADDR_X430_MODEL_INFO              (2)
#define ADDR_X430_FIRMWARE_VERSION        (4)
#define ADDR_X430_ID                      (7)
#define ADDR_X430_BAUDRATE                (8)
#define ADDR_X430_RETURN_TIME_DELAY       (9)
#define ADDR_X430_DRIVE_MODE              (10)
#define ADDR_X430_OPERATING_MODE          (11)
#define ADDR_X430_SHADOW_ID               (12)
#define ADDR_X430_PROTOCOL_VERSION        (13)
#define ADDR_X430_HOMING_OFFSET           (20)
#define ADDR_X430_MOVING_THRESHOLD        (24)
#define ADDR_X430_TEMPERATURE_LIMIT       (31)
#define ADDR_X430_MAX_VOLTAGE_LIMIT       (32)
#define ADDR_X430_MIN_VOLTAGE_LIMIT       (34)
#define ADDR_X430_MAX_PWM_LIMIT           (36)
#define ADDR_X430_MAX_PWM_LIMIT           (36)
#define ADDR_X430_MAX_CURRENT_LIMIT       (38)
#define ADDR_X430_MAX_ACCELERATION_LIMIT  (40)
#define ADDR_X430_MAX_VELOCITY_LIMIT      (44)
#define ADDR_X430_MAX_POSITION_LIMIT      (48)
#define ADDR_X430_MIN_POSITION_LIMIT      (52)
#define ADDR_X430_SHUTDOWN                (63)

#define ADDR_X430_TORQUE_ENABLE           (64)
#define ADDR_X430_LED                     (65)
#define ADDR_X430_STATUS_RETURN_LEVEL     (68)
#define ADDR_X430_REGISTERED              (69)
#define ADDR_X430_HARDWARE_ERROR_STAUS    (70)
#define ADDR_X430_VELOCITY_I_GAIN         (76)
#define ADDR_X430_VELOCITY_P_GAIN         (78)
#define ADDR_X430_POSITION_D_GAIN         (80)
#define ADDR_X430_POSITION_I_GAIN         (82)
#define ADDR_X430_POSITION_P_GAIN         (84)
#define ADDR_X430_FEEDFORWARD2_GAIN       (88)
#define ADDR_X430_FEEDFORWARD1_GAIN       (90)
#define ADDR_X430_BUS_WATCHDOG            (98)
#define ADDR_X430_GOAL_PWM                (100)
#define ADDR_X430_GOAL_CURRENT            (102)
#define ADDR_X430_GOAL_VELOCITY           (104)
#define ADDR_X430_PROFILE_ACCELERATION    (108)
#define ADDR_X430_PROFILE_VELOCITY        (112)
#define ADDR_X430_GOAL_POSITION           (116)
#define ADDR_X430_REALTIME_TICK           (120)
#define ADDR_X430_MOVING                  (122)
#define ADDR_X430_MOVING_STATUS           (123)
#define ADDR_X430_PRESENT_PWM             (124)
#define ADDR_X430_PRESENT_LOAD            (126)
#define ADDR_X430_PRESENT_VELOCITY        (128)
#define ADDR_X430_PRESENT_POSITION        (132)
#define ADDR_X430_VELOCITY_TRAJECTORY     (136)
#define ADDR_X430_POSITION_TRAJECTORY     (140)
#define ADDR_X430_PRESENT_VOLTAGE         (144)
#define ADDR_X430_PRESENT_TEMPERATURE     (146)

/****************************************************************************************/
// Host Rx Buffer States                                                                */
/****************************************************************************************/
#define sizeRxDFRobotMsg                  (13)            // Size of DFRobot Joystick Command Message
#define sizeRxLE3DMsg                     (25)            // Size of LE3D Joystick Command Message
#define HOSTCOMM_HEADER1                  (255)           // Header 1 
#define HOSTCOMM_HEADER2                  (255)           // Heafer 2
#define HOSTCOMM_HEADER3                  (254)           // Heafer 3
#define sizeTmBuffer                      (30)            // Size of TM Buffer
#define TM_HEADER1                        (255)           // Header 1 
#define TM_HEADER2                        (253)           // Heafer 2
byte    comTmBuffer[sizeTmBuffer];                        // TM Buffer
/****************************************************************************************/
// Gun Instruction Set                                                                  */
/****************************************************************************************/
#define ID_GUN                            (110)           // GUN ID
#define COMM_ARM                          (65)            // ASCII "A"
#define COMM_DISARM                       (68)            // ASCII "D"
#define COMM_FIRE                         (70)            // ASCII "F"
#define COMM_BURST                        (66)            // ASCII "B"
#define COMM_HOPPER_LOAD                  (72)            // ASCII "H"
#define COMM_HOPPER_JAM                   (74)            // ASCII "J"
#define COMM_TEST                         (84)            // ASCII "T"
/****************************************************************************************/
// Pin Definitions                                                                      */
/****************************************************************************************/
#define pinA9                             (9)
#define pinD13                            (13)
#define pinD14                            (14)
#define pinD22                            (22)
#define pinD23                            (23)
#define pinCamera                         (18)
#define pinPowrMain                       (2)
#define powrfMainCutOff                   (3.1*3)
#define pinTargetFront                    (15)
#define pinTargetLeft                     (10)
#define pinTargetRear                     (16)
#define pinTargetRight                    (17)
/****************************************************************************************/
// Leg Definitions                                                                      */
/****************************************************************************************/
#define DxlNull                           (511)        // Dynamixel Null Position      (cnts)
//#define DxlNull                           (2045)        // Dynamixel Null Position      (cnts)
#define DxlNull_AX12                      (511)        // Dynamixel Null Position      (cnts)

// Body Dimensions Front Legs
#define L0F         (1.47875984251969F) // Shoulder Fixed Length        (in)
#define L1F         (2.97602362204724F) // Upper Leg (Femur) Length     (in)
#define L2F         (5.97283031496063F) // Lower Leg (Tibia) Length     (in)

#define PhiF        (.371374125299082F) // Shoulder Fixed Angle         (rad)
#define RSHF        (3.93700787401575F) // Shoulder Joint Radius        (in)
#define HSHF        (5.76257637795276F-.50F) // Shoulder Joint Height        (in)
#define LFTFDEFAULT (4.33598188976378F-.25F) // Foot Radius - Joint Radius   (in)
#define LFTF        (4.33598188976378F+.25F) // Foot Radius - Joint Radius   (in)

// Body Dimensions Rear Legs
#define L0R         (1.47875984251969F) // Shoulder Fixed Length        (in)
#define L1R         (2.97602362204724F) // Upper Leg (Femur) Length     (in)
#define L2R         (5.97283031496063F) // Lower Leg (Tibia) Length

#define PhiR        (.371374125299082F) // Shoulder Fixed Angle         (rad)
#define RSHR        (3.93700787401575F) // Shoulder Joint Radius        (in)
#define HSHR        (5.76257637795276F-.50F) // Shoulder Joint Height        (in)
#define LFTRDEFAULT (4.33598188976378F-.25F) // Foot Radius - Joint Radius   (in) 
#define LFTR        (4.33598188976378F+.2F) // Foot Radius - Joint Radius   (in) 

// Offsets used in calculation of Dynamixel angle commands  
#define offsetT00   (-45.0000F)         // Leg 0, Theta 0               (deg)             
#define offsetT10   (-135.0000F)        // Leg 1, Theta 0               (deg)
#define offsetT20   (135.0000F)         // Leg 2, Theta 0               (deg)
#define offsetT30   (45.0000F)          // Leg 3, Theta 0               (deg)

// Compensate Bracket Offsets
#define offsetTF1   (-27.58203000000F)  // Front Legs - Theta 1 Offest  (deg) 
#define offsetTR1   (-27.58203000000F)  // Rear Legs -  Theta 1 Offest  (deg)
#define offsetTF2   (-62.41797000000F)  // Front Legs - Theta 2 Offset  (deg)
#define offsetTR2   (-62.41797000000F)  // Rear Legs -  Theta 2 Offset  (deg)

// Compensate for Curved Legs
#define offsetT02   (-7.00000000000F)   // Leg 0, Theta 2 Offset        (deg)     
#define offsetT12   (-7.00000000000F)   // Leg 1, Theta 2 Offset        (deg)
#define offsetT22   (-7.00000000000F)   // Leg 2, Theta 2 Offset        (deg)
#define offsetT32   (-7.00000000000F)   // Leg 3, Theta 2 Offset        (deg)
/****************************************************************************************/
// Leg Angle Limits                                                                     */
/****************************************************************************************/
#define Joint1CWlimF    (0)            // Front Joint 1 CW Angle Limit  (cnts)
//#define Joint1CCWlimF   (4095)         // Front Joint 1 CCW Angle Limit (cnts)
#define Joint1CCWlimF   (1023)         // Front Joint 1 CCW Angle Limit (cnts)
#define Joint1CWlimR    (0)            // Rear Joint 1 CW Angle Limit  (cnts)
//#define Joint1CCWlimR   (4095)         // Rear Joint 1 CCW Angle Limit (cnts)
#define Joint1CCWlimR   (1023)         // Rear Joint 1 CCW Angle Limit (cnts)

#define Joint2CWlimF    (0)            // Front Joint 2 CW Angle Limit  (cnts)
//#define Joint2CCWlimF   (4095)         // Front Joint 2 CCW Angle Limit (cnts)
#define Joint2CCWlimF   (1023)         // Front Joint 2 CCW Angle Limit (cnts)
#define Joint2CWlimR    (0)            // Rear Joint 2 CW Angle Limit  (cnts)
//#define Joint2CCWlimR   (4095)         // Rear Joint 2 CCW Angle Limit (cnts)
#define Joint2CCWlimR   (1023)         // Rear Joint 2 CCW Angle Limit (cnts)

#define Joint3CWlimF    (0)            // Front Joint 3 CW Angle Limit  (cnts)
//#define Joint3CCWlimF   (4095)         // Front Joint 4 CCW Angle Limit (cnts)
#define Joint3CCWlimF   (1023)         // Front Joint 4 CCW Angle Limit (cnts)
#define Joint3CWlimR    (0)            // Rear Joint 3 CW Angle Limit  (cnts)
//#define Joint3CCWlimR   (4095)         // Rear Joint 4 CCW Angle Limit (cnts)
#define Joint3CCWlimR   (1023)         // Rear Joint 4 CCW Angle Limit (cnts)
/****************************************************************************************/
// Turret Definitions                                                                   */
/****************************************************************************************/
#define AZ1         (51)               // AZ1 Turret Dynamixel ID
#define AZ2         (52)               // AZ2 Turret Dynamixel ID
#define EL1         (61)               // EL1 Turret Dynamixel ID
#define EL2         (62)               // EL2 Turret Dynamixel ID
#define MELEE1      (71)               // Melee Weapon Dynamixel ID
#define AZbias      (5)                // AZ Turret Command Bias   (cnts)
//#define AZMinLim    (0+AZbias)         // AZ CW Angle Limit        (cnts)
//#define AZMaxLim    (4095-AZbias)      // AZ CCW Angle Limit       (cnts)
//#define ELbias      (0)                // EL Turret Command Bias   (cnts)
//#define ELMinLim    (2047-200)         // EL CW Angle Limit        (cnts)
//#define ELMaxLim    (2047+200)         // EL CCW Angle Limit       (cnts)
#define AZMinLim    (435+AZbias)         // AZ CW Angle Limit        (cnts)
#define AZMaxLim    (530-AZbias)       // AZ CCW Angle Limit       (cnts)
#define ELbias      (0)                 // EL Turret Command Bias   (cnts)
#define ELMinLim    (435)                 // EL CW Angle Limit        (cnts)
#define ELMaxLim    (530)              // EL CCW Angle Limit       (cnts)
/****************************************************************************************/
/* End Mech Define Header                                                               */
/****************************************************************************************/
#endif 
