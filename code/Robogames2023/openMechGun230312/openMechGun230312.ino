// Built with Arduino IDE 1.6.5
// Board Arduio Mini Pro
// Processor Atmega328 (5V, 16MHz)
// Hopper Declarations

#include <SPI.h>

// Hopper Declarations
#define pinHopper2Dir 8           // Hopper PWM Pin
#define pinHopper2PWM 9           // Hopper PWM Pin

// Gun Declarations
#define pinGun2PWM    3           // Gun PWM Pin
#define pinGun2In1    4           // Gun PWM Pin
#define pinGun2In2    7           // Gun PWM Pin

// Miscellaneous Declarations
#define pinLaser        2         // Digital Laser Pin
#define pinLED          5         // LED
#define pinPowerPlane2  A7        // 
#define pinA0           A0        // Analog
#define pinA1           A1        // Analog
#define pinA2           A2        // Analog
#define pinA3           A3        // Analog

// Uncomment TimerFive for Atmega2560. Also need to Uncomment
#include <TimerOne.h>
#define TimerMS1        1000      // 1000 Hz
boolean ISRState        = false;  // ISR Interrupt Flag
int ISRcount            = 0;      // ISR Interrupt Flag

// Arming and Firing Declarations
boolean valArmState         = LOW;   // Arm State (LOW = Disarmed, HIGH = Armed)
boolean valArmStartupState  = LOW;
int     valArmLoopCount     = 0;
int     valArmCount         = 0;
boolean valArmToggle        = LOW;

// Gun Test Declarations
boolean valTestState        = LOW;
int valTestCount            = 0;

// Gun Fire Declarations
boolean valFireState1       = LOW;
int valFireCount1           = 0;
int valFireloopCount1       = 1;
int valBurst                = 1;
boolean valToggleState      = LOW;

int valJam                  = 0;
boolean valJamState         = LOW;

int valLoad                 = 0;
boolean valLoadState        = LOW;

// Communication Declarations
#define commRxSize 5                // Rx Buffer Size
byte commBufferSerial[5]    = {0};  // Rx Buffer
byte commIdxSerial          = 0;    // Comm Buffer Index
byte commInstruction        = 0;    // Comm Instruction

// Circular Buffer States
#define COMMSTATE_HEADER1     0 // Header 1
#define COMMSTATE_HEADER2     1 // Header 2
#define COMMSTATE_ID          2 // Identification
#define COMMSTATE_INSTRUCTION 3 // Instruction
#define COMMSTATE_CHKSUM      4 // Checksum

// Instruction Set
#define COMM_ARM           65   // ASCII "A"
#define COMM_DISARM        68   // ASCII "D"
#define COMM_FIRE          70   // ASCII "F"
#define COMM_BURST         66   // ASCII "B"
#define COMM_HOPPER_LOAD   72   // ASCII "H"
#define COMM_HOPPER_JAM    74   // ASCII "J"
#define COMM_TEST          84   // ASCII "T"

#define commHEADER1 103  //
#define commHEADER2 117  //
#define commID      110  //

// Function Prototypes
void fcn_LoadHopper(int loopCount);
void GunRx(void);
void fcn_FlushBuffer(void);
void fcn_Jam(void);

void GunArmSR(void);
void GunTestSR(void);
void GunFireSR1(void);

// SPI Variables
char commBufferSPI [100];
volatile byte pos;
volatile boolean process_it;

void setup() {
  pinMode(pinHopper2Dir, OUTPUT);
  pinMode(pinHopper2PWM, OUTPUT);

  pinMode(pinHopper2Dir,  OUTPUT);
  pinMode(pinHopper2PWM,  OUTPUT);
  pinMode(pinGun2PWM,     OUTPUT);
  pinMode(pinGun2In1,     OUTPUT);
  pinMode(pinGun2In2,     OUTPUT);
  pinMode(pinLaser,       OUTPUT);
  pinMode(pinLED,         OUTPUT);
  pinMode(pinPowerPlane2, INPUT);
  pinMode(pinA0,          INPUT);
  pinMode(pinA1,          INPUT);
  pinMode(pinA2,          INPUT);
  pinMode(pinA3,          INPUT);

  //  digitalWrite(pinLED,HIGH);
  //  digitalWrite(pinLaser,HIGH);
  //  digitalWrite(pinGun2In1,HIGH);
  //  delay(1000);
  //  digitalWrite(pinGun2In1,LOW);
  //  digitalWrite(pinGun2In2,HIGH);
  //  delay(1000);
  //  digitalWrite(pinGun2In2,LOW);

  Serial.begin(115200);
  //  Serial.begin(1000000);
  delay(100);
  fcn_FlushBuffer();
  delay(100);
  fcn_LoadHopper(5);

  // SPI Setup
  // turn on SPI in slave mode
  SPCR |= bit (SPE);
  // have to send on master in, *slave out*
  pinMode(MOSI, INPUT);
  pinMode(MISO, OUTPUT);
  //  pinMode(MISO, INPUT);
  pinMode(SCK, INPUT);
  // get ready for an interrupt
  pos = 0;   // buffer empty
  process_it = false;
  // now turn on interrupts
  SPI.attachInterrupt();

  Timer1.initialize(TimerMS1);         // initialize timer1, and set a 100K usecond period
  Timer1.attachInterrupt(ISRTimer1);  // attaches callback() as a timer overflow interrupt
}

void loop() {
  int commCHKSUM;
  //***********************************************************************
  // SPI BUS
  //***********************************************************************
  if (process_it) {
    commCHKSUM = 0;
    for (int i = 2; i <= commRxSize - 2; i++) {
      commCHKSUM = commCHKSUM + int(commBufferSPI[i]);
    }

    commCHKSUM = 255 - lowByte(commCHKSUM);
    if (int(commBufferSPI[0]) == commHEADER1 &&
        int(commBufferSPI[1]) == commHEADER2 &&
        int(commBufferSPI[2]) == commID      &&
        int(commBufferSPI[4]) == commCHKSUM) {
      commInstruction = commBufferSPI[3];
    }
    else {
      commInstruction = 0;
    }
    // Serial.println(commInstruction);
    commBufferSPI [pos] = 0;
    pos = 0;
    process_it = false;
  }
  //***********************************************************************
  // Read Gun Board Rx if Available
  //***********************************************************************
  if (Serial.available() > 0) {
    GunRx();
  }

  if (commInstruction == COMM_ARM)
  {
    //    Serial.println(commInstruction);
    digitalWrite(pinLED, HIGH);
    valArmState = HIGH;
    valArmStartupState = HIGH;
    valArmCount = 1;
    commInstruction = 0;
    fcn_FlushBuffer();
  }
  else if (commInstruction == COMM_DISARM && valArmStartupState == LOW)
  {
    valArmState = LOW;
    commInstruction = 0;
    digitalWrite(pinLaser, LOW);
    digitalWrite(pinLED, LOW);
    fcn_FlushBuffer();

  }
  else if (commInstruction == COMM_TEST && valArmStartupState == LOW)
  {
    valTestState = HIGH;
    valTestCount = 1;
    commInstruction = 0;
    fcn_FlushBuffer();
  }
  else if (commInstruction == COMM_FIRE && valArmStartupState == LOW && valArmState == HIGH)
  {
    if (valFireState1 == LOW)
    {
      valFireState1 = HIGH;
      valFireCount1 = 1;
      valBurst = 1;
      commInstruction = 0;
    }
    commInstruction = 0;
    fcn_FlushBuffer();
  }
  else if (commInstruction == COMM_BURST && valArmStartupState == LOW && valArmState == HIGH)
  {
    valFireState1 = HIGH;
    valFireCount1 = 1;
    valBurst = 10;
    commInstruction = 0;
    fcn_FlushBuffer();
  }
  else if (commInstruction == COMM_HOPPER_JAM)
  {
    //Serial.println(commInstruction);
    valJamState = HIGH;
    valJam = 1;
    commInstruction = 0;
    fcn_FlushBuffer();
  }
  else if (commInstruction == COMM_HOPPER_LOAD)
  {
    //Serial.println(commInstruction);
    valLoadState = HIGH;
    valLoad = 1;
    commInstruction = 0;
    fcn_FlushBuffer();
  }
  else
  {
  }

  GunArmSR();
  GunTestSR();
  GunFireSR1();
  fcn_Jam();
  fcn_LoadHopper(2);

  if (ISRState)
  {
    valToggleState = !valToggleState;
    // Reset ISR Flag
    ISRState = false;
  }
}
/*********************************************************************************************/
/* Gun Arm Service Routine                                                                   */
/*********************************************************************************************/
void GunArmSR() {
  // Flash Laser when Armed.
  //  Serial.println(valArmStartupState);
  if (valArmStartupState == HIGH & ISRState == HIGH)
  {
    // Load Hopper
    digitalWrite(pinHopper2Dir, LOW);
    analogWrite(pinHopper2PWM, 128);

    if (valArmCount > 100)
    {
      valArmCount = 1;
      valArmLoopCount = valArmLoopCount + 1;
    }

    if (valArmCount == 1)
    {
      digitalWrite(pinLaser, HIGH);
    }
    else if (valArmCount == 25)
    {
      digitalWrite(pinLaser, LOW);
    }

    if (valArmLoopCount > 5)
    {
      valArmCount = 0;
      valArmLoopCount = 0;
      valArmStartupState = LOW;
      digitalWrite(pinLaser, LOW);
      // Turn off hooper
      digitalWrite(pinHopper2Dir, LOW);
      analogWrite(pinHopper2PWM, 0);
    }
    else
    {
      valArmCount = valArmCount + 1;
    }
  }
}
/*********************************************************************************************/
/* Gun Test Service Routine                                                                  */
/*********************************************************************************************/
void GunTestSR() {
  if (valTestState == HIGH & ISRState == HIGH)
  {
    // Test Gun
    analogWrite(pinHopper2PWM, 175);
    analogWrite(pinGun2PWM, 175);
    digitalWrite(pinGun2In1, LOW);
    digitalWrite(pinGun2In2, HIGH);

    if (valTestCount > 100)
    {
      digitalWrite(pinLaser, LOW);
      valTestCount = 0;
      valTestState = LOW;

      analogWrite(pinHopper2PWM, 0);
      analogWrite(pinGun2PWM, 0);
      digitalWrite(pinGun2In1, HIGH);
      digitalWrite(pinGun2In2, LOW);

      fcn_FlushBuffer();
    }
    else
    {
      digitalWrite(pinLaser, HIGH);
      valTestCount = valTestCount + 1;
    }
  }
}
/*********************************************************************************************/
/* Gun 1 Fire Service Routine                                                                  */
/*********************************************************************************************/
void GunFireSR1() {
  if (valFireState1 == HIGH & ISRState == HIGH)
  {
    if (valFireCount1 == 1)
    {
      // Forward Hopper
      digitalWrite(pinHopper2Dir, HIGH);
      analogWrite(pinHopper2PWM, 255);
      //      digitalWrite(pinLED, HIGH);
    }
    else if (valFireCount1 == 50)
    {
      // Cock Gun
      digitalWrite(pinGun2In1, LOW);
      digitalWrite(pinGun2In2, HIGH);
//      analogWrite(pinGun2PWM, 90);
     analogWrite(pinGun2PWM, 110);
    }
    else if (valFireCount1 == 100)
    {
      // Fire Gun
      analogWrite(pinGun2PWM, 255);
    }
    else if (valFireCount1 == 180)
    {
      // Reverse Hopper
      digitalWrite(pinHopper2Dir, LOW);
      analogWrite(pinHopper2PWM, 255);

      // Turn off gun
      analogWrite(pinGun2PWM, 0);
    }
    else if (valFireCount1 == 195) {
      // Forward Hopper
      digitalWrite(pinHopper2Dir, HIGH);
      analogWrite(pinHopper2PWM, 255);
    }
    else if (valFireCount1 == 210)
    {
      //Turn Off Hopper
      analogWrite(pinHopper2PWM, 0);
      //      digitalWrite(pinLED, LOW);
    }
    else if (valFireCount1 == 330)
    {
      if (valFireloopCount1 == valBurst)
      {
        valFireState1 = LOW;
        valFireloopCount1 = 0;
      }
      valFireloopCount1 = valFireloopCount1 + 1;
      valFireCount1 = 0;
    }
    valFireCount1 = valFireCount1 + 1;
  }
}
/*********************************************************************************************/
// SPI interrupt routine
/*********************************************************************************************/
ISR (SPI_STC_vect)
{
  byte c = SPDR;  // grab byte from SPI Data Register

    Serial.println (c);

  // add to buffer if room
  if (pos < (sizeof (commBufferSPI) - 1))
    commBufferSPI[pos++] = c;

  // example: newline means time to process buffer
  if (c == '\n')
    process_it = true;
}  // end of interrupt routine SPI_STC_vect
/*********************************************************************************************
   Gun Rx Function - From Main Robot Controller
 *********************************************************************************************/
void GunRx() {
  // Instruction Packet : 0XFF    0XFF    0X01 0X01        0XFB`
  //                      HEADER1 HEADER2 ID   INSTRUCTION CHECKSUM

  // commIdxSerial Definitions
  //    0 = Awaiting Header 1
  //    1 = Awaiting Header 2
  //    2 = Awaiting ID
  //
  //  #define commHEADER1 103  //
  //  #define commHEADER2 117  //
  //  #define commID      110  //

  byte commMsg = Serial.read();
  int commCHKSUM;

  if ((commIdxSerial == COMMSTATE_HEADER1))
  {
    // If Valid Header1
    if (commMsg == commHEADER1)
    {
      commIdxSerial = COMMSTATE_HEADER2;
      commBufferSerial[COMMSTATE_HEADER1] = commMsg;
    }
    else {
      fcn_FlushBuffer();
    }
  }
  else if (commIdxSerial == COMMSTATE_HEADER2)
  {
    if (commMsg == commHEADER2)
    {
      commIdxSerial = COMMSTATE_ID;
      commBufferSerial[COMMSTATE_HEADER2] = commMsg;
    }
    else
    {
      commIdxSerial = COMMSTATE_HEADER1;
    }
  }
  else if (commIdxSerial == COMMSTATE_ID)
  {
    if (commMsg == commID)
    {
      commIdxSerial = COMMSTATE_INSTRUCTION;
      commBufferSerial[COMMSTATE_ID] = commMsg;
    }
    else
    {
      commIdxSerial = COMMSTATE_HEADER1;
    }
  }
  else if (commIdxSerial == COMMSTATE_INSTRUCTION)
  {
    commIdxSerial = COMMSTATE_CHKSUM;
    commBufferSerial[COMMSTATE_INSTRUCTION] = commMsg;
  }
  else if (commIdxSerial == COMMSTATE_CHKSUM)
  {
    commBufferSerial[COMMSTATE_CHKSUM] = commMsg;

    commCHKSUM = 0;
    for (int i = 2; i <= commRxSize - 2; i++) {
      commCHKSUM = commCHKSUM + int(commBufferSerial[i]);
    }
    commCHKSUM = 255 - lowByte(commCHKSUM);

    commIdxSerial = COMMSTATE_HEADER1;
    if (commCHKSUM == commBufferSerial[COMMSTATE_CHKSUM])
    {
      commInstruction = commBufferSerial[COMMSTATE_INSTRUCTION];
      //Serial.println(commInstruction);
    }
  }
  else
  {
    commIdxSerial = COMMSTATE_HEADER1;
    commInstruction = 0;
  }
}
//***********************************************************************
//***********************************************************************
//***********************************************************************
//***********************************************************************
// Load Hopper Function
//***********************************************************************
void fcn_LoadHopper(int loopCount) {
  if (valLoadState == HIGH & ISRState == HIGH) {
    if (valLoad == 1)
    {
      // Turn On Hopper High
      digitalWrite(pinHopper2Dir, HIGH);
      analogWrite(pinHopper2PWM, 255);
    }
    else if (valLoad == 100 * loopCount)
    {
      // Turn On Hopper High
      digitalWrite(pinHopper2Dir, LOW);
      analogWrite(pinHopper2PWM, 0);
      valLoadState = LOW;
      valLoad = 0;
    }
    valLoad = valLoad + 1;
  }
}
//***********************************************************************
//***********************************************************************
//***********************************************************************
//***********************************************************************
// Hopper Jam Function
//***********************************************************************
void fcn_Jam(void)
{
  if (valJamState == HIGH & ISRState == HIGH) {
    if (valJam == 1)
    {
      // Turn On Hopper High
      digitalWrite(pinHopper2Dir, LOW);
      analogWrite(pinHopper2PWM, 255);
    }
    else if (valJam == 100)
    {
      // Turn On Hopper High
      digitalWrite(pinHopper2Dir, HIGH);
      analogWrite(pinHopper2PWM, 255);
    }
    else if (valJam == 200)
    {
      // Turn On Hopper High
      digitalWrite(pinHopper2Dir, LOW);
      analogWrite(pinHopper2PWM, 255);
    }
    else if (valJam == 300)
    {
      // Turn On Hopper High
      digitalWrite(pinHopper2Dir, HIGH);
      analogWrite(pinHopper2PWM, 255);
    }
    else if (valJam == 400)
    {
      // Turn On Hopper High
      digitalWrite(pinHopper2Dir, LOW);
      analogWrite(pinHopper2PWM, 255);
    }
    else if (valJam == 500)
    {
      // Turn On Hopper High
      digitalWrite(pinHopper2Dir, LOW);
      analogWrite(pinHopper2PWM, 0);
      valJamState = LOW;
      valJam = 0;
    }
    valJam = valJam + 1;
  }
}
//***********************************************************************
//***********************************************************************
//***********************************************************************
// Flush (Clear) Rx Buffer Function
//***********************************************************************
void fcn_FlushBuffer(void) {
  // Flush Buffer
  while (Serial.available() > 0)
  {
    Serial.read();
  }
}

/*********************************************************************************************
   Timer Interrupt Service Routine
 *********************************************************************************************/
void ISRTimer1() {
  //void ISRTimer5(){ // Uncomment for Atmega2560
  ISRState = true;
}
