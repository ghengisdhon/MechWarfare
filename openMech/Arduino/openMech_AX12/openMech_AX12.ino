/****************************************************************************************/
/* Mech Control Program                                                               */
/* Written by D. Croft                                                                  */
/*                                                                                      */
/* Compatible with Arduino 1.8.5                                                        */
/*                                                                                      */
/* Releases:                                                                            */
/* v4.00 Initial Release for Arduino IDE                                                */ 
/****************************************************************************************/

/****************************************************************************************/
/* Libraries                                                                            */
/****************************************************************************************/
#include <math.h>
#include <SPI.h>
#include <HardwareTimer.h> // Need to Install dynamixel_servo from managed libraries
#define CIRCULAR_BUFFER_XS  // Reduce the library indexes to unsigned short 
#define CIRCULAR_BUFFER_INT_SAFE
#include <CircularBuffer.h> 
#include <DynamixelSDK.h>  

/****************************************************************************************/
/* Custom Mech Header Files                                                             */
/****************************************************************************************/
#include "mechDefine.h"
#include "mechGlobal.h"
#include "mechMath.h"
#include "mechDxlAX12.h"
#include "mechDxlXL320.h"
#include "mechDxlX430.h"
#include "mechUtilities.h"
#include "mechKinematics.h"
#include "mechGait.h" 
#include "mechComm.h"

/****************************************************************************************/
/* Initializations                                                                      */
/****************************************************************************************/
boolean ISRFlag  = false;                       // Main ISR Interrupt Flag
void ISRTimer();                                // Main Interrupt Service Routine
boolean ISRWDTFlag  = false;                    // Watch Dog Timer ISR Interrupt Flag
void ISRWDT();                                  // Watch Dog Timer Interrupt Service Routine
boolean ISRToggle = false;

void TargetFrontISR();
void TargetLeftISR();
void TargetRearISR();
void TargetRightISR();
//boolean TargetFrontFlag = false;
//boolean TargetLeftFlag = false;
//boolean TargetRearFlag = false;
//boolean TargetRightFlag = false;

void setup()
{ 
  AX12.begin(DXL_BUS_SERIAL1,AX12_PROTOCOL_VERSION,DXL_BUS_BAUDRATE);
  XL320.begin(DXL_BUS_SERIAL1,XL320_PROTOCOL_VERSION,DXL_BUS_BAUDRATE);
  X430.begin(DXL_BUS_SERIAL1,X430_PROTOCOL_VERSION,DXL_BUS_BAUDRATE);
  delay(100);

  // Initialize Mech Global Variables and Dynamixels
  initMech();   
  initAX12();
//  initX430();
  initTurret();
  
  /* SPI Ports Setup                                                                    */
  // SPI1 Port - (Not Used)
  SPI_1.beginTransaction(SPISettings(140625, MSBFIRST, SPI_MODE0));

  // SPI2 Port - Gun Communications
  SPI_2.beginTransaction(SPISettings(140625, MSBFIRST, SPI_MODE0));

  // Setup Serial Ports
  Serial3.begin(baudXbee);
  SerialUSB.begin(115200);
  comRxHostBuffer.clear();
  
  // Setup Digital Pins
  pinMode(BOARD_LED_PIN, OUTPUT); // built-in LED pin as an output Used for Debugging
  pinMode(13, OUTPUT);
  pinMode(pinTargetFront, INPUT_PULLUP);
  pinMode(pinTargetLeft,  INPUT_PULLUP);
  pinMode(pinTargetRear,  INPUT_PULLUP);
  pinMode(pinTargetRight, INPUT_PULLUP);

  // Attach Target Plate Interrupts
  attachInterrupt(digitalPinToInterrupt(pinTargetFront), TargetFrontISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(pinTargetLeft),  TargetLeftISR,  FALLING);
  attachInterrupt(digitalPinToInterrupt(pinTargetRear),  TargetRearISR,  FALLING);
  attachInterrupt(digitalPinToInterrupt(pinTargetRight), TargetRightISR, FALLING);
  
  // Initialize Main Timer
  Timer.pause();
  Timer.setPeriod(TimerMS2);        
  Timer.attachInterrupt(ISRTimer);
  Timer.refresh();
  Timer.resume();  
}

/****************************************************************************************/
/* Main Processor Loop                                                                  */
/************************************************************;;;****************************/
void loop() {
  HostRx();

//  /**************************************************************************************/
//  /* Service the Target Plate ISRs                                                      */
//  /**************************************************************************************/
//  if(mechGV.TargetFrontFlag){
////    SerialUSB.print("Front = \t");
////    SerialUSB.print(mechGV.TargetFrontFlag);
////    SerialUSB.print("\n");
//    mechGV.TargetFrontFlag = false;
//  }
//    if(mechGV.TargetLeftFlag){
////    SerialUSB.print("Left = \t");
////    SerialUSB.print(mechGV.TargetLeftFlag);
////    SerialUSB.print("\n");
//    mechGV.TargetLeftFlag = false;
//  }
//    if(mechGV.TargetRearFlag){
////    SerialUSB.print("Rear = \t");
////    SerialUSB.print(mechGV.TargetRearFlag);
////    SerialUSB.print("\n");
//    mechGV.TargetRearFlag = false;
//  }
//    if(mechGV.TargetRightFlag){
////    SerialUSB.print("Right = \t");
////    SerialUSB.print(mechGV.TargetRightFlag);
////    SerialUSB.print("\n");
//    mechGV.TargetRightFlag = false;
//  }

  /**************************************************************************************/
  /* Service the Timer ISR                                                              */
  /**************************************************************************************/
  if (ISRFlag) {
    mechGV.commWDT++; 
    
    // Check battery Voltages
    ChkBattery();
    
    // Set LED Pin High for Timing Measurment (can use an oscillosope to measure) 
    digitalWrite(13,ISRToggle);

    // Determine Gait (Step Positions)
    Gait();           //3.9 msec -> 2.6 msec with trig lookups -> 1.6 msec further optimization

    // Send Communication Commands
    GaitTx();         // Gait Command Transmit    (1.6 msec -> 1.0 msec using SynchWrite)
    GunTx();          // Gun Command Transmit     (4.0 usec)
    TurretTx();       // Turret Command Transmit  (2.1 msec -> 0.7 msec using SynchWrite)
    MechTmTx();       // Telemetry Transmit

    //
    mechGaitControl(); // Handle Gait State Changes
    mechBodyControl();
    mechTurretControl();
    mechGunControl(); 

  /**************************************************************************************/
  /* Service the Target Plate ISRs                                                      */
  /**************************************************************************************/
  if(mechGV.TargetFrontFlag){
//    SerialUSB.print("Front = \t");
//    SerialUSB.print(mechGV.TargetFrontFlag);
//    SerialUSB.print("\n");
    mechGV.TargetFrontFlag = false;
  }
    if(mechGV.TargetLeftFlag){
//    SerialUSB.print("Left = \t");
//    SerialUSB.print(mechGV.TargetLeftFlag);
//    SerialUSB.print("\n");
    mechGV.TargetLeftFlag = false;
  }
    if(mechGV.TargetRearFlag){
//    SerialUSB.print("Rear = \t");
//    SerialUSB.print(mechGV.TargetRearFlag);
//    SerialUSB.print("\n");
    mechGV.TargetRearFlag = false;
  }
    if(mechGV.TargetRightFlag){
//    SerialUSB.print("Right = \t");
//    SerialUSB.print(mechGV.TargetRightFlag);
//    SerialUSB.print("\n");
    mechGV.TargetRightFlag = false;
  }

  // Set LED Pin Low for Timing Measurment
//    digitalWrite(BOARD_LED_PIN, LOW);
     
    // Resetr ISR Flag
    ISRToggle = !ISRToggle;
    ISRFlag = false;
  }
}

/****************************************************************************************/
/* Timer Interrupt Service Routine                                                      */
/****************************************************************************************/
void ISRTimer() {
  ISRFlag = true;
}

/****************************************************************************************/
/* Target Plate Interrupt Service Routines                                              */
/****************************************************************************************/
void TargetFrontISR() {
  mechGV.TargetFrontFlag = true;
}

void TargetLeftISR() {
  mechGV.TargetLeftFlag = true;
}

void TargetRearISR() {
  mechGV.TargetRearFlag = true;
}

void TargetRightISR() {
  mechGV.TargetRightFlag = true;
}
