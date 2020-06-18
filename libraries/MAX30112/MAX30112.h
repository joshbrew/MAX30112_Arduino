// MAX30112 Lib by Joshua Brewster (MIT License)
// This is a modifed Sparkfun Electronics MAX30105 library (Beerware) - https://github.com/sparkfun/SparkFun_MAX3010x_Sensor_Library


#pragma once

#if (ARDUINO >= 100)
 #include "Arduino.h"
#else
 #include "WProgram.h"
#endif

#include <Wire.h>

#define MAX30112_ADDRESS          0x60 //0b1100000
#define MAX30112_WRITE            0xC1
#define MAX30112_READ             0xC0

#define I2C_SPEED_STANDARD        100000
#define I2C_SPEED_FAST            400000

//Define the size of the I2C buffer based on the platform the user has
#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__)

  //I2C_BUFFER_LENGTH is defined in Wire.H
  #define I2C_BUFFER_LENGTH BUFFER_LENGTH

#elif defined(__SAMD21G18A__)

  //SAMD21 uses RingBuffer.h
  #define I2C_BUFFER_LENGTH SERIAL_BUFFER_SIZE

#else

  //The catch-all default is 32
  //#define I2C_BUFFER_LENGTH 32

#endif

// Status Registers
static const uint8_t MAX30112_INTSTAT1   =		      0x00;
static const uint8_t MAX30112_INTSTAT2   =		      0x01;
static const uint8_t MAX30112_INTENABLE1 =		      0x02;
static const uint8_t MAX30112_INTENABLE2 =		      0x03;

// FIFO Registers
static const uint8_t MAX30112_FIFO_WR_PTR = 	      0x04;
static const uint8_t MAX30112_OVF_COUNTER = 	      0x05;
static const uint8_t MAX30112_FIFO_RD_PTR = 	      0x06;
static const uint8_t MAX30112_FIFO_DATA   =		      0x07;

// Configuration Registers
static const uint8_t MAX30112_FIFOCONFIG         =  0x08;
static const uint8_t MAX30112_DATCTRL1           =  0x09;
static const uint8_t MAX30112_DATCTRL2           = 	0x0A;    
static const uint8_t MAX30112_SYSCTRL            = 	0x0D;
static const uint8_t MAX30112_PPG_CFG1           = 	0x0E;
static const uint8_t MAX30112_PPG_CFG2           =  0x0F;
static const uint8_t MAX30112_LED_PROX_THRESHOLD = 	0x10;
static const uint8_t MAX30112_LED1_PA            =  0x11;
static const uint8_t MAX30112_LED2_PA            =  0x12;
static const uint8_t MAX30112_LED_RGE            =  0x14;
static const uint8_t MAX30112_LED_PILOT_PA       =  0x15;

// Part ID
static const uint8_t MAX30112_PARTID  =             0xFF;

//Bit Masks and Commands by Register

//MAX30112_INTSTAT1/INTENABLE1
static const uint8_t MAX30112_A_FULL_EN_MASK    =  (byte)~0b10000000;
static const uint8_t MAX30112_PPG_RDY_EN_MASK   =  (byte)~0b1000000;
static const uint8_t MAX30112_ALC_OVF_EN_MASK   =  (byte)~0b100000;
static const uint8_t MAX30112_PROX_INT_EN_MASK  =  (byte)~0b10000;
static const uint8_t MAX30112_LED_COMPB_EN_MASK =  (byte)~0b1000;

static const uint8_t MAX30112_A_FULL_ENABLE    =  (byte)~0b10000000;
static const uint8_t MAX30112_PPG_RDY_ENABLE   =  (byte)~0b1000000;
static const uint8_t MAX30112_ALC_OVF_ENABLE   =  (byte)~0b100000;
static const uint8_t MAX30112_PROX_INT_ENABLE  =  (byte)~0b10000;
static const uint8_t MAX30112_LED_COMPB_ENABLE =  (byte)~0b1000;

//MAX30112_INTSTAT2/INTENABLE2
static const uint8_t MAX30112_VDD_OOR_EN_MASK =  (byte)~0b10000000; //Checks that the analog-in voltage is within 1.65V-2.05V
static const uint8_t MAX30112_VDD_OOR_ENABLE  =  (byte)~0b10000000;



//MAX30112_FIFOCONFIG
static const uint8_t MAX30112_FIFO_STAT_CLR_MASK    =  (byte)~0b01000000;
static const uint8_t MAX30112_FIFO_A_FULL_TYPE_MASK =  (byte)~0b00100000;
static const uint8_t MAX30112_FIFO_RO_MASK          =  (byte)~0b00010000;
static const uint8_t MAX30112_FIFO_A_FULL_MASK      =  (byte)~0b00001111;

static const uint8_t MAX30112_FIFO_STAT_CLR  =  (byte)~0b01000000; //Set if reading FIFO clears the A_FULL Interrupt
static const uint8_t MAX30112_FIFO_AFULLONCE =  (byte)~0b00100000; //Set A_FULL to assert only once when a full sample buffer is detected (default every time)
static const uint8_t MAX30112_FIFO_RO        =  (byte)~0b00010000; //Set FIFO to automatically roll over on full

//Set which sample the A_FULL Interrupt is triggered by.
static const uint8_t MAX30112_FIFO_A_FULL_32 =  (byte)~0b0000;
static const uint8_t MAX30112_FIFO_A_FULL_31 =  (byte)~0b0001;
static const uint8_t MAX30112_FIFO_A_FULL_17 =  (byte)~0b1111;



//MAX30112_DATCTRL1/DATCTRL2
static const uint8_t MAX30112_FD1_MASK =  (byte)~0b00001111;
static const uint8_t MAX30112_FD2_MASK =  (byte)~0b11110000;
static const uint8_t MAX30112_FD3_MASK =  (byte)~0b00001111;
static const uint8_t MAX30112_FD4_MASK =  (byte)~0b11110000;

//Write these to the FIFO Data Registers to set what data they will be filled with. None stops this sample cycle. You may have up to 4 samples per cycle * 32 sample cycles in the FIFO
static const uint8_t MAX30112_FD_MODE_NONE    =  0x0;
static const uint8_t MAX30112_FD_MODE_LED1    =  0x1; //LED 1 sample (contains automatic ambient cancellation)
static const uint8_t MAX30112_FD_MODE_LED2    =  0x2; //LED 2 sample
static const uint8_t MAX30112_FD_MODE_AMBIENT =  0xC; //Direct Ambient sample
static const uint8_t MAX30112_FD_MODE_LED1P2  =  0xD; //LED 1 + 2 sample



//MAX30112_SYSCTRL
static const uint8_t MAX30112_FCLK_CTRL_MASK =  (byte)~0b00010000;  
static const uint8_t MAX30112_LP_MODE_MASK   =  (byte)~0b00001000;  
static const uint8_t MAX30112_FIFO_EN_MASK   =  (byte)~0b00000100;  
static const uint8_t MAX30112_SHDN_MASK      =  (byte)~0b00000010; 
static const uint8_t MAX30112_RESET_MASK     =  (byte)~0b00000001;   

static const uint8_t MAX30112_FCLK_CTRL =  (byte)~0b10000; //Can use external 32kHz clock when PPG SR is 100sps or lower for device synchronization purposes
static const uint8_t MAX30112_LP_MODE   =  (byte)~0b1000;  //Low Power mode setting for 100sps or lower sample rate.
static const uint8_t MAX30112_FIFO_EN   =  (byte)~0b100;   //Setting this bit enables the use of the FIFO data buffer, flushes the FIFO and loads new samples from the 0 pointer.
static const uint8_t MAX30112_SHDN      =  (byte)~0b10;    //Power saving mode, all registers retain values with read/write functionality. Clears interrupts.
static const uint8_t MAX30112_RESET     =  (byte)~0b1;     //Force power reset. Resets all registers.



//MAX30112_PPG_CFG1
static const uint8_t MAX30112_PPG_ADC_RGE_MASK = (byte)~0b11000000;
static const uint8_t MAX30112_PPG_SR_MASK =     (byte)~0b00111100;
static const uint8_t MAX30112_PPG_TINT_MASK =   (byte)~0b00000011;

//MAX30112_ADC_RGE - ADC Range (MAX = 11.4pA LSB, HIGH = 22.9pA LSB, LOW = 45.8pA LSB, MIN = 91.6pA LSB)
static const uint8_t MAX30112_PPG_ADC_RGE_MAX  =  0x00;
static const uint8_t MAX30112_PPG_ADC_RGE_HIGH =  0x40;
static const uint8_t MAX30112_PPG_ADC_RGE_LOW  =  0x80;
static const uint8_t MAX30112_PPG_ADC_RGE_MIN  =  0xC0;

//MAX30112_PPG_SR - Sampling Rate
//Single Pulse Mode
static const uint8_t MAX30112_PPG_SR_20   =  0x0;
static const uint8_t MAX30112_PPG_SR_25   =  0x4;
static const uint8_t MAX30112_PPG_SR_50   =  0x8;
static const uint8_t MAX30112_PPG_SR_84   =  0xC;
static const uint8_t MAX30112_PPG_SR_100  =  0x10;
static const uint8_t MAX30112_PPG_SR_200  =  0x14;
static const uint8_t MAX30112_PPG_SR_400  =  0x18;
static const uint8_t MAX30112_PPG_SR_800  =  0x1C;
static const uint8_t MAX30112_PPG_SR_1000 =  0x20;
static const uint8_t MAX30112_PPG_SR_1600 =  0x24;
static const uint8_t MAX30112_PPG_SR_3200 =  0x28;

//Dual Pulse Mode
static const uint8_t MAX30112_PPG_SR_D20  =  0x2C;
static const uint8_t MAX30112_PPG_SR_D25  =  0x30;
static const uint8_t MAX30112_PPG_SR_D50  =  0x34;
static const uint8_t MAX30112_PPG_SR_D84  =  0x38;
static const uint8_t MAX30112_PPG_SR_D100 =  0x3C;

//MAX30112_PPG_TINT - Pulse Width (microseconds)
static const uint8_t MAX30112_TINT_52  = 0x0; //16 Bit Resolution
static const uint8_t MAX30112_TINT_104 = 0x1; //17 Bit Resolution
static const uint8_t MAX30112_TINT_206 = 0x2; //18 Bit Resolution
static const uint8_t MAX30112_TINT_417 = 0x3; //19 Bit Resolution

//MAX30112_PPG_CFG2
static const uint8_t MAX30112_LED_SETLNG_MASK = (byte)~0b00011000;
static const uint8_t MAX30112_SMP_AVE_MASK    = (byte)~0b00000111;

//MAX30112_LED_SETLNG -- LED settling time (microseconds)
static const uint8_t MAX30112_LED_SETLNG_2_5 = 0x0; 
static const uint8_t MAX30112_LED_SETLNG_5   = 0x8;
static const uint8_t MAX30112_LED_SETLNG_10  = 0x10;
static const uint8_t MAX30112_LED_SETLNG_20  = 0x18;

//MAX30112_SMP_AVE
static const uint8_t MAX30112_SMP_AVE_1  = 0x0;
static const uint8_t MAX30112_SMP_AVE_2  = 0x1;
static const uint8_t MAX30112_SMP_AVE_4  = 0x2;
static const uint8_t MAX30112_SMP_AVE_8  = 0x3;
static const uint8_t MAX30112_SMP_AVE_16 = 0x4;
static const uint8_t MAX30112_SMP_AVE_32 = 0x5;



//MAX30112_LED_PROX_THRESHOLD -- Sets the proximity threshold to quit sampling. 0x01 = 2048 (decimal) or the maximum threshold by which the the interrupt is triggered
static const uint8_t MAX30112_PROX_MIN =  0x01; // Minimal saturation
static const uint8_t MAX30112_PROX_MED =  0x80; // Half saturated.
static const uint8_t MAX30112_PROX_MAX =  0xFF; // Fully saturated.



//MAX30112_LED1_PA/LED2_PA settings. This is the percentage of the drive current used to power the LEDs. 255 = 100% of MAX30112_LED_RGE
static const uint8_t MAX30112_LED_OFF  =  0x00; //0
static const uint8_t MAX30112_LED_MIN  =  0x01; //1
static const uint8_t MAX30112_LED_LOW  =  0x20; //32
static const uint8_t MAX30112_LED_MED  =  0x80; //128
static const uint8_t MAX30112_LED_HIGH =  0xC8; //200
static const uint8_t MAX30112_LED_MAX  =  0xFF; //255



//MAX30112_LED_RGE -- set each LED current (milliAmps)
static const uint8_t MAX30112_LED1_RGE_MASK =  (byte)~0b00000011;
static const uint8_t MAX30112_LED2_RGE_MASK =  (byte)~0b00001100;

static const uint8_t MAX30112_LED_RGE_50  = 0x00; //LED current in milliAmps
static const uint8_t MAX30112_LED_RGE_100 = 0x05;
static const uint8_t MAX30112_LED_RGE_150 = 0x0A;
static const uint8_t MAX30112_LED_RGE_200 = 0xF;

//MAX30112_LED_PILOT_PA -- Use the LED PA settings. Current (in milliAmps) settings for proximity mode (see docs)

static const uint8_t MAX30112_DISABLE = 0x00; //Generic disable byte



class MAX30112 {
 public: 
  MAX30112(void);

  boolean begin(TwoWire &wirePort = Wire, uint32_t i2cSpeed = I2C_SPEED_STANDARD, uint8_t i2caddr = MAX30112_ADDRESS);

  uint32_t getRed(void); //Returns immediate red value
  uint32_t getIR(void); //Returns immediate IR value
  bool     safeCheck(uint8_t maxTimeToCheck); //Given a max amount of time, check for new data

  // Configuration
  void softReset();
  void shutDown(); 
  void wakeUp(); 

  void setLEDMode(uint8_t fd1=MAX30112_FD_MODE_LED1, uint8_t fd2=MAX30112_FD_MODE_LED2, uint8_t fd3=MAX30112_FD_MODE_AMBIENT, uint8_t fd4=MAX30112_FD_MODE_NONE);

  void setADCRange(uint8_t adcRange);
  void setSampleRate(uint8_t sampleRate);
  void setPulseWidth(uint8_t pulseWidth);

  void setPulseAmplitudeRed(uint8_t power, uint8_t value);
  void setPulseAmplitudeIR(uint8_t power, uint8_t value);

  //FIFO Reading
  uint16_t check(void); //Checks for new data and fills FIFO
  uint8_t  available(void); //Tells caller how many new samples are available (head - tail)
  void     nextSample(void); //Advances the tail of the sense array
  uint32_t getFIFORed(void); //Returns the FIFO sample pointed to by tail
  uint32_t getFIFOIR(void); //Returns the FIFO sample pointed to by tail

  uint8_t  getWritePointer(void);
  uint8_t  getReadPointer(void);
  uint8_t  getOverflowCounter(void);
  void     clearFIFO(void); //Sets the read/write pointers to zero

  uint8_t  readPartID();

  
  // Setup the IC with user selectable settings
  void setup( uint8_t ledMode = 3, uint8_t LED_PA = MAX30112_LED_RGE_50, uint8_t LED_RGE = 0xFF, uint8_t sampleAverage = MAX30112_SMP_AVE_4, uint8_t sampleRate = MAX30112_PPG_SR_1000, uint8_t pulseWidth = MAX30112_TINT_417, uint8_t ledSettling = MAX30112_LED_SETLNG_20, uint8_t adcRange = MAX30112_PPG_ADC_RGE_MAX);
  void setupLPmode(); //Low power setting

  // Low-level I2C communication
  uint8_t readRegister8(uint8_t address, uint8_t reg);
  void writeRegister8(uint8_t address, uint8_t reg, uint8_t value);

 private:
  TwoWire *_i2cPort; //The generic connection to user's chosen I2C hardware
  uint8_t _i2caddr;

  //activeLEDs is the number of channels turned on, and can be 1 to 3. 2 is common for Red+IR.
  byte activeLEDs; //Gets set during setup. Allows check() to calculate how many bytes to read from FIFO
  
  void bitMask(uint8_t reg, uint8_t mask, uint8_t thing);
 
  #define STORAGE_SIZE 4 //Each long is 4 bytes so limit this to fit on your micro
  typedef struct Record
  {
    uint32_t red[STORAGE_SIZE];
    uint32_t IR[STORAGE_SIZE];
    uint32_t amb[STORAGE_SIZE];
    uint32_t l4[STORAGE_SIZE];
    byte head;
    byte tail;
  } sense_struct; //This is our circular buffer of readings from the sensor

  sense_struct sense;

};
