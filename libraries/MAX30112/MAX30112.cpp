#include "MAX30112.h"
  
MAX30112::MAX30112() {
      //Constructor
  }

boolean MAX30112::begin(TwoWire &wirePort = Wire, uint32_t i2cSpeed = I2C_SPEED_STANDARD, uint8_t i2creadaddr = MAX30112_READ_ADDRESS, uint8_t i2cwriteaddr = MAX30112_WRITE_ADDRESS) {

    _i2cPort = &wirePort; //Grab which port the user wants us to use

    _i2cPort->begin();
    _i2cPort->setClock(i2cSpeed);

    _i2creadaddr = i2creadaddr;
    _i2cwriteaddr = i2cwriteaddr;

    // Step 1: Initial Communication and Verification
    // Check that a MAX30112 is connected
    // Set modes for the MAX30112
    
  }

uint32_t MAX30112::getRed(void){
    //Check the sensor for new data for 250ms
    if(safeCheck(250))
      return (sense.red[sense.head]);
    else
      return(0); //Sensor failed to find new data
  } //Returns immediate red value

uint32_t MAX30112::getIR(void){
    //Check the sensor for new data for 250ms
    if(safeCheck(250))
      return (sense.IR[sense.head]);
    else
      return(0); //Sensor failed to find new data
  } //Returns immediate IR value

bool MAX30112::safeCheck(uint8_t maxTimeToCheck){
    uint32_t markTime = millis();
    
    while(1)
    {
    if(millis() - markTime > maxTimeToCheck) return(false);

    if(check() == true) //We found new data!
      return(true);

    delay(1);
    }
  } //Given a max amount of time, check for new data

  // Configuration
void MAX30112::softReset(){
    bitMask(MAX30112_SYSCTRL, MAX30112_RESET_MASK, MAX30112_RESET);
  }

void MAX30112::shutDown(){
    bitMask(MAX30112_SYSCTRL, MAX30112_SHDN_MASK, MAX30112_SHDN);   
  }

void MAX30112::wakeUp(){
    bitMask(MAX30112_SYSCTRL, MAX30112_SHDN_MASK, MAX30112_DISABLE);
  }

void MAX30112::setLEDMode(uint8_t fd1=MAX30112_FD_MODE_LED1, uint8_t fd2=MAX30112_FD_MODE_LED2, uint8_t fd3=MAX30112_FD_MODE_NONE, uint8_t fd4=MAX30112_FD_MODE_NONE){
      bitMask(MAX30112_DATCTRL1, MAX30112_FD1_MASK, fd1);
      bitMask(MAX30112_DATCTRL1, MAX30112_FD2_MASK, fd2);
      bitMask(MAX30112_DATCTRL2, MAX30112_FD3_MASK, fd3);
      bitMask(MAX30112_DATCTRL2, MAX30112_FD4_MASK, fd4);
  }

void MAX30112::setADCRange(uint8_t adcRange){ //Use MAX30112_PPG_ADC_RGE_XXXX commands
    bitMask(MAX30112_PPG_CFG1, MAX30112_PPG_ADC_RGE_MASK, adcRange);
  }

void MAX30112::setSampleRate(uint8_t sampleRate){ //Use MAX30112_PPG_SR_XXXX commands
    bitMask(MAX30112_PPG_CFG1, MAX30112_PPG_SR_MASK, sampleRate);
  }

void MAX30112::setPulseWidth(uint8_t pulseWidth){ //Use MAX30112_TINT_XXX commands
    bitMask(MAX30112_PPG_CFG1, MAX30112_PPG_TINT_MASK, pulseWidth);
  }

void MAX30112::setPulseAmplitudeRed(uint8_t power, uint8_t value){ //Set max power and percentage of that power (0-255 = 0%-100%) to pulse LEDs
    writeRegister8(_i2cwriteaddr, MAX30112_LED1_PA, power);
    bitMask(MAX30112_LED_RGE, MAX30112_LED1_RGE_MASK, value);
  }

void MAX30112::setPulseAmplitudeIR(uint8_t power, uint8_t value){
    writeRegister8(_i2cwriteaddr, MAX30112_LED2_PA, power);
    bitMask(MAX30112_LED_RGE,MAX30112_LED2_RGE_MASK,value);
  }

//FIFO Reading
//Polls the sensor for new data
//Call regularly
//If new data is available, it updates the head and tail in the main struct
//Returns number of new samples obtained
uint16_t MAX30112::check(void)
{
  //Read register FIDO_DATA in (3-byte * number of active LED) chunks
  //Until FIFO_RD_PTR = FIFO_WR_PTR

  byte readPointer = getReadPointer();
  byte writePointer = getWritePointer();

  int numberOfSamples = 0;

  //Do we have new data?
  if (readPointer != writePointer)
  {
    //Calculate the number of readings we need to get from sensor
    numberOfSamples = writePointer - readPointer;
    if (numberOfSamples < 0) numberOfSamples += 32; //Wrap condition

    //We now have the number of readings, now calc bytes to read
    //For this example we are just doing Red and IR (3 bytes each)
    int bytesLeftToRead = numberOfSamples * activeLEDs * 3;

    //Get ready to read a burst of data from the FIFO register
    //_i2cPort->beginTransmission(_i2creadaddr);
    //_i2cPort->write(MAX30112_FIFODATA);
    //_i2cPort->endTransmission();

    //We may need to read as many as 288 bytes so we read in blocks no larger than I2C_BUFFER_LENGTH
    //I2C_BUFFER_LENGTH changes based on the platform. 64 bytes for SAMD21, 32 bytes for Uno.
    //Wire.requestFrom() is limited to BUFFER_LENGTH which is 32 on the Uno
    while (bytesLeftToRead > 0)
    {
      int toGet = bytesLeftToRead;
      if (toGet > I2C_BUFFER_LENGTH)
      {
        //If toGet is 32 this is bad because we read 6 bytes (Red+IR * 3 = 6) at a time
        //32 % 6 = 2 left over. We don't want to request 32 bytes, we want to request 30.
        //32 % 9 (Red+IR+GREEN) = 5 left over. We want to request 27.

        toGet = I2C_BUFFER_LENGTH - (I2C_BUFFER_LENGTH % (activeLEDs * 3)); //Trim toGet to be a multiple of the samples we need to read
      }

      bytesLeftToRead -= toGet;

      //Request toGet number of bytes from sensor
      _i2cPort->requestFrom(_i2creadaddr, toGet);
      
      while (toGet > 0)
      {
        sense.head++; //Advance the head of the storage struct
        sense.head %= STORAGE_SIZE; //Wrap condition

        byte temp[sizeof(uint32_t)]; //Array of 4 bytes that we will convert into long
        uint32_t tempLong;

        //Burst read three bytes - RED
        temp[3] = 0;
        temp[2] = _i2cPort->read();
        temp[1] = _i2cPort->read();
        temp[0] = _i2cPort->read();

        //Convert array to long
        memcpy(&tempLong, temp, sizeof(tempLong));
		
		tempLong &= 0x3FFFF; //Zero out all but 18 bits

        sense.red[sense.head] = tempLong; //Store this reading into the sense array

        if (activeLEDs > 1)
        {
          //Burst read three more bytes - IR
          temp[3] = 0;
          temp[2] = _i2cPort->read();
          temp[1] = _i2cPort->read();
          temp[0] = _i2cPort->read();

          //Convert array to long
          memcpy(&tempLong, temp, sizeof(tempLong));

		  tempLong &= 0x3FFFF; //Zero out all but 18 bits
          
		  sense.IR[sense.head] = tempLong;
        }

        if (activeLEDs > 2)
        {
          //Burst read three more bytes - Green
          temp[3] = 0;
          temp[2] = _i2cPort->read();
          temp[1] = _i2cPort->read();
          temp[0] = _i2cPort->read();

          //Convert array to long
          memcpy(&tempLong, temp, sizeof(tempLong));

		  tempLong &= 0x3FFFF; //Zero out all but 18 bits

          sense.green[sense.head] = tempLong;
        }

        toGet -= activeLEDs * 3;
      }

    } //End while (bytesLeftToRead > 0)

  } //End readPtr != writePtr

  return (numberOfSamples); //Let the world know how much new data we found
}

uint8_t MAX30112::available(void){
      
  }//Tells caller how many new samples are available (head - tail)

void MAX30112::nextSample(void){
    if(available()) //Only advance the tail if new data is available
    {
      sense.tail++;
      sense.tail %= STORAGE_SIZE; //Wrap condition
    }
  } //Advances the tail of the sense array

uint32_t MAX30112::getFIFORed(void){
    return (sense.red[sense.tail]);
  } //Returns the FIFO sample pointed to by tail

uint32_t MAX30112::getFIFOIR(void){
    return (sense.IR[sense.tail]);
  }//Returns the FIFO sample pointed to by tail

//Read the FIFO Write Pointer
uint8_t MAX30112::getWritePointer(void) {
    return (readRegister8(_i2creadaddr, MAX30112_FIFO_WR_PTR));
  }

//Read the FIFO Read Pointer
uint8_t MAX30112::getReadPointer(void) {
    return (readRegister8(_i2creadaddr, MAX30112_FIFO_RD_PTR));
  }

  //Advance the tail
void MAX30112::nextSample(void)
{
  if(available()) //Only advance the tail if new data is available
  {
    sense.tail++;
    sense.tail %= STORAGE_SIZE; //Wrap condition
  }
}

void MAX30112::clearFIFO(void) {
  writeRegister8(_i2cwriteaddr, MAX30112_FIFO_WR_PTR, 0x00);
  writeRegister8(_i2cwriteaddr, MAX30112_OVF_COUNTER, 0x00);
  writeRegister8(_i2cwriteaddr, MAX30112_FIFO_RD_PTR, 0x00);
} //Sets the read/write pointers to zero

uint8_t MAX30112::readPartID(){
  return (readRegister8(_i2creadaddr, MAX30112_PARTID));
  }

// Setup the IC with user selectable settings
void MAX30112::setup(byte powerLevel = 0x1F, byte sampleAverage = 4, byte ledMode = 3, int sampleRate = 400, int pulseWidth = 411, int adcRange = 4096){
    
  }

  // Low-level I2C communication
//Given a register, read it, mask it, and then set the thing
void MAX30112::bitMask(uint8_t reg, uint8_t mask, uint8_t thing)
{
  // Grab current register context
  uint8_t originalContents = readRegister8(_i2creadaddr, reg);

  // Zero-out the portions of the register we're interested in
  originalContents = originalContents & mask;

  // Change contents
  writeRegister8(_i2cwriteaddr, reg, originalContents | thing);
}

//
// Low-level I2C Communication
//
uint8_t MAX30112::readRegister8(uint8_t address, uint8_t reg) {
  _i2cPort->beginTransmission(address);
  _i2cPort->write(reg);
  _i2cPort->endTransmission(false);

  _i2cPort->requestFrom((uint8_t)address, (uint8_t)1); // Request 1 byte
  if (_i2cPort->available())
  {
    return(_i2cPort->read());
  }

  return (0); //Fail

}

void MAX30112::writeRegister8(uint8_t address, uint8_t reg, uint8_t value) {
  _i2cPort->beginTransmission(address);
  _i2cPort->write(reg);
  _i2cPort->write(value);
  _i2cPort->endTransmission();
}

 