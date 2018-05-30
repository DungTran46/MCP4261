
// MCP4261 2-channel Digital Potentiometer
// ww1.microchip.com/downloads/en/DeviceDoc/22059b.pdf

// The default SPI Control Register - SPCR = B01010000;
// interrupt disabled,spi enabled,msb 1st,master,clk low when idle,
// sample on leading edge of clk,system clock/4 rate (fastest).
// Enable the digital pins 11-13 for SPI (the MOSI,MISO,SPICLK)
#include <SPI.h>

#include "MCP4261.h"

//---------- constructor ----------------------------------------------------
MCP4261::MCP4261(uint8_t slave_select, uint8_t sck, uint8_t sdi, uint8_t sdo)
{
	//setup slave select pin
	slave_select_pin=slave_select;
	pinMode(slave_select_pin,OUTPUT);
	digitalWrite(slave_select_pin,HIGH);
	
	//setup other pins
	sck_pin=sck;
	sdi_pin=sdi;
	sdo_pin=sdo;
	
	//calculate rs
	rs=RAB/RESOLUTION_8BITS;
}

//------------------ protected -----------------------------------------------

//get wiper postion with a desired Rwa
uint16_t MCP4261::get_wiper_pos(int rwa)
{
	uint16_t num_rs;
	if(rwa<TYPICAL_RW)
		num_rs=TYPICAL_RW; //smallest resistance 
	else if (rwa<RAB)
		num_rs=(rwa-TYPICAL_RW)/rs;
	else
		num_rs=0;	// largeest resistance RAB
	return num_rs;
}
/*
	Function constructs commandByte
	memory_address: desired memory address from 0x00 to 0x0F
	command_bits: define type of operation 
		0b00: write
		0b11: read
	refer to page 45 of MCP4261 datasheet if need more info
*/
uint16_t MCP4261::contruct_commandByte(uint8_t memory_address, uint8_t command_bits)
{
	uint16_t temp=0x0000;
	temp=(temp|(memory_address << 4)|command_bits)<<8;
	return temp;
}

uint16_t MCP4261:: construct_dataByte(int rwa)
{
	return get_wiper_pos(rwa);
}

uint16_t MCP4261:: command(uint16_t commandByte, uint16_t dataByte )
{
	uint16_t temp= commandByte|MASK;
	return (commandByte|MASK)|(dataByte|MASK);
}

uint16_t MCP4261::write(uint8_t address, int rwa, bool isVolatile)
{
	Serial.println();
	uint16_t commandByte=contruct_commandByte(address,CMD_WRITE);
	uint16_t dataByte=construct_dataByte(rwa);
	uint16_t package= command(commandByte, dataByte );
	Serial.println(commandByte);
	Serial.println(dataByte);
	Serial.println(package);
	digitalWrite(slave_select_pin, LOW);
	uint16_t feedback = SPI.transfer(package);
	if(!isVolatile)
	{
		delay(5);
	}
	digitalWrite(slave_select_pin, HIGH);
}

/*uint16_t MCP4261::read(byte cmd_byte)
{
  cmd_byte |= kCMD_READ;
  ::digitalWrite(slave_select_pin, LOW);
  byte high_byte = SPI.transfer(cmd_byte);
  byte low_byte  = SPI.transfer(0xFF);
  ::digitalWrite(slave_select_pin, HIGH);
  return byte2uint16(high_byte, low_byte);
}*/

/*void MCP4261::wiper_pos(byte pot, unsigned int wiper_pos)
{
  byte cmd_byte    = 0x00;
  byte data_byte   = 0x00;
  cmd_byte        |= pot;

  // Calculate the 9-bit data value to send
  if(wiper_pos > 255)
    cmd_byte      |= B00000001; // Table 5-1 (page 36)
  else
    data_byte      = (byte)(wiper_pos & 0x00FF);

  write(cmd_byte|kADR_VOLATILE, data_byte);

  if(non_volatile)
  {
    // EEPROM write cycles take 4ms each. So we block with delay(5); after any NV Writes
    write(cmd_byte|kADR_NON_VOLATILE, data_byte);
    delay(5);
  }
}*/

//---------- public ----------------------------------------------------
void MCP4261:: begin()
{
	SPI.begin(sck_pin, sdo_pin, sdi_pin, slave_select_pin);
	SPI.setClockDivider(SPI_CLOCK_DIV32);
	SPI.setBitOrder(MSBFIRST);
	SPI.setDataMode(SPI_MODE0);
}
float MCP4261:: getRs()
{
	return rs;
}

uint16_t MCP4261::set_wiper0(int rwa)
{
  return write(NON_VOLATILE_WIPER_0,rwa,true);
}

