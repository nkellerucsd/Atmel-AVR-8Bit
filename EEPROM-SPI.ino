/*
 * Example code that writes to an EEPROM chip using the SPI communication
 * protocol.
 * 
 * See EEPROM chip user guide for details on SPI transmission protocol.
 * EEPROM chip: Microchip 25LC256
 * 
 */
 
#include <avr/io.h>
#include <avr/interrupt.h>

 /*
  * EEPROM Chip Defines
  */

// # of bytes of per a single page
// Note: address must be updated after a single page read/write 
#define BytesPerPage 64

// slave select and deselect macros
#define SLAVE_SELECT PORTB &= ~(1<<PB2); // pull low
#define SLAVE_DESELECT PORTB |= (1<<PB2) // pull high


// define 8-bit commands for the EEPROM device
#define EEPROM_READ 0b00000011 /* read memory */
#define EEPROM_WRITE 0b00000010 /* write to memory */
#define EEPROM_WRDI 0b00000100 /* write disable */
#define EEPROM_WREN 0b00000110 /* write enable */
#define EEPROM_RDSR 0b00000101 /* read status register */
#define EEPROM_WRSR 0b00000001 /* write status register */

// EEPROM Status Register Bits -- from data sheet
// Use these to parse status register
#define EEPROM_WRITE_IN_PROGRESS 0
#define EEPROM_WRITE_ENABLE_LATCH 1
#define EEPROM_BLOCK_PROTECT_0 2
#define EEPROM_BLOCK_PROTECT_1 3
#define EEPROM_BYTES_PER_PAGE 64
#define EEPROM_BYTES_MAX 0x7FFF

// Master (AVR) bits
#define MOSI PB3
#define MISO PB4
#define SS PB2
#define SCK PB5

// Slave (EEPROM) bits
#define WIP 0 // write in progess (bit is cleared after a write cycle is done)

/*
 * Functions
 */
 
// Load some random data
void loaddata(uint8_t data[])
{
uint8_t i = 0;
for(i=0;i<BytesPerPage;i++)
  data[i] = i;
}

// EEPROM chip functions


// write N elements to EE prom chip
void writePage(uint16_t address, uint8_t array[])
{  
  uint8_t junk = 0, i=0;
  // enable write
  writeEnable();

  SLAVE_SELECT;
  
  // write instruction to eeprom
  junk = tradeByte(EEPROM_WRITE);

  // select address to begin writing to
  junk = tradeByte((uint8_t)address>>8);
  junk = tradeByte((uint8_t)address);

  // begine to write data
  for(i=0;i<BytesPerPage;i++)
  {
    tradeByte(array[i]);
  }

  // new instruction --> check WIP bit
  SLAVE_DESELECT;
  
  // wait until WIP is cleared
  while( readStatus() & (1<<WIP) )
  ;

}

// read page
void readPage(uint16_t address, uint8_t out[])
{
  uint8_t junk = 0,i = 0;
  
  SLAVE_SELECT;
  
  junk = tradeByte(EEPROM_READ);
  
  // select address to read from
  junk = tradeByte((uint8_t)address>>8);
  junk = tradeByte((uint8_t)address);
  
  // begine to read data
  for(i=0;i<BytesPerPage;i++)
  {
    out[i] = tradeByte(0);
  }  

  SLAVE_DESELECT;
}

void sendByte(uint16_t address, uint8_t byte)
{
  uint8_t rx = 0, junk = 0; 

  writeEnable();
  
  SLAVE_SELECT;

  // write instruction
  junk = tradeByte(EEPROM_WRITE);

  // select address
  junk = tradeByte((uint8_t)address>>8);
  junk = tradeByte((uint8_t)address);

  // write data to that address
  junk = tradeByte(byte);

  SLAVE_DESELECT;

  // check WIP bit to see if still writing
  // stop when WIP bit is clear (WIP = write in progress)
  while( ( readStatus() & (1<<WIP) ) )
  ; 
}


void writeEnable(void)
{
  SLAVE_SELECT;
  tradeByte(EEPROM_WREN);
  SLAVE_DESELECT;
}



uint8_t readByte(uint16_t address)
{
  uint8_t rx = 0, junk = 0; 
  SLAVE_SELECT;
  junk = tradeByte(EEPROM_READ);
  // send HB and then LB
  junk = tradeByte((uint8_t)address>>8);
  junk = tradeByte((uint8_t)address);

  // clock in junk to return data byte at address
  rx = tradeByte(0);
  SLAVE_DESELECT;
  return(SPDR);
}


uint8_t readStatus()
{
  uint8_t rx = 0, junk = 0; 
  SLAVE_SELECT;
  junk = tradeByte(EEPROM_RDSR); 
  rx = tradeByte(0);
  SLAVE_DESELECT;
  return(rx);
}


uint8_t tradeByte(uint8_t mosi)
{
  // clock in 8 bits
  SPDR = mosi; 

  // Note SPIF is cleared when (1) SPIF is read AND (2) recieve register is read
  // loop_until_bit_is_set(SPSR,SPIF);
  // wait until SPIF bit in SPSR register is set
  // AND selects SPIF bit and stops when SPIF == 1
  while( !( SPSR & (1<<SPIF) ) )
  ;

  return(SPDR); // return contents in data register
}

void initSPI(void)
{
  // set up ports (all output except MISO)
  DDRB |= (1<<MOSI) | (1<<SS) | (1<<SCK);  
  DDRB &= ~(1<<MISO); // input
  PORTB |= (1<<MISO); // enable pull-up --> high (doesn't matter SO is floating when not enabled)

  // start by pulling SS high (deselect)
  PORTB |= (1<<SS);
  
  // set up SPI control register
  SPCR |= (1<<MSTR); // AVR is master, EEPROM is slave
  SPCR |= (1<<SPR1); // SPI clock rate 8/64 MHz = 0.125 MHz
  SPCR |= (1<<SPE); // enable SPI  
}

/*
 * UART functions
 */

void initUART(void)
{  
  UBRR0H = 0;                        
  UBRR0L = 103;

  /* 8 data bits, 1 stop bit */
  UCSR0B |= (1 << TXEN0) | (1 << RXEN0);
  UCSR0C |= (1 << UCSZ01) | (1 << UCSZ00);   

  // enable ISR for when unread data is in read-buffer 
  UCSR0B |= (1<<RXCIE0);

}


void printPage(uint8_t data[])
{
  uint8_t i = 0;
  for(i=0;i<BytesPerPage;i++)
  {
    printByte(data[i]);
    transmitByte('\n');
  }
}



void transmitByte(uint8_t data)
{
  loop_until_bit_is_set(UCSR0A,UDRE0);  
  
  // copy byte to transmit buffer
  UDR0 = data;
}


void printByte(uint8_t byte){
/* Converts a byte to a string of decimal text, sends it */
transmitByte('0'+ (byte/100)); /* Hundreds */
transmitByte('0'+ ((byte/10) % 10)); /* Tens */
transmitByte('0'+ (byte % 10)); /* Ones */
}



/*
 * Enter Main loop
 */



int main(void)
{
  initUART();
  initSPI();

  // test if chip read/write process works
  // write a byte, then read
  // from it at same address
  uint16_t address = 0, page = 0;
  uint8_t data = 43, rx = 0;
  sendByte(address, data);
  rx = readByte(address);

  // 43 should be printed on terminal
  printByte(rx);

  // load a 64 byte array w/ elements = 0,1,2,...,63
  loaddata(data);

  // write three pages
  // must write 64 bytes per page
  // update address after a page write
  for(page = 0; page < 3; page++) {
  writePage(address, data);
  address += page*BytesPerPage;
  }
  
  // read three pages
  // again, address must be updated after a page read
  page = 0;
  address = 0;
  
  for(page = 0; page < 3; page++) {
  readPage(address, data);
  address += page*BytesPerPage;
  printPage(data);
  }
  
while(1)
{
  ;
}

return(0);
}



