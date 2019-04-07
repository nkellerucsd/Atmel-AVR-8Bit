/*
 * Digitize 2 channels with 10 bit resolution from
 * an external transducer. The transducer DC power supply is used as
 * the reference voltage (AREF). Therefore, the REFS bits 0 and 1 are left
 * as 0 so as to not short the ADC internal AVCC reference voltage.
 * 
 * Place a 100 nF capacitor between the AREF (connected to (+) DC supply
 * of transducer) and ground (connected to (-) of DC supply and to GND of board)
 * 
 * Two channels are digitized at a specified sampling rate. The 16 bit timer is 
 * used as the trigger source for the ADC. After a conversion is complete, the high and low
 * byte are stored in a FIFO buffer. The channel # is also stored in the highst bits
 * of the high byte.
 * 
 * The 10-bit result is popped from the buffer and printed on the serial terminal screen.
 */


#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

void initTimer1(void);
void initADC(void);
void initUART(void);

#define BUF_SIZE 64  // use a FIFO buffer to ensure data is not lost
#define INC 2 // increment pointers by 2 since 2 bytes (LB & HB) are pushed/popped per operation
#define NUM_CHANNELS 2 // digitizing three channels

// channel bit selection --> change last 2 bits of ADMUX register
#define CHANNEL_MASK ( (1<<0) | (1<<1) ) // 0,...0,1,1

// FIFO buffer
volatile uint8_t buffer[BUF_SIZE] = {0};

// head -- push operation and tail -- pop operation 
// full -- checked when pushing,  empty -- checked when popping 
volatile uint8_t head = 0, tail = 0, firstTime = 0, mux = 0;

// ISR trigger after ADC completes a conversion (ADIF flag is set)
ISR(ADC_vect)
{ 
  uint8_t hb = 0;
  
  // PUSH 8-bit data into buffer instruction
  // also, check if buffer is FULL
  
  // clear ADC trigger flag, which triggers ADC conversion 
  // Note that ADC conversion time = 13 ADC clk ticks,
  // and ADC conversion triggered by timer1 every second

  // channel register selection can be updated BEFORE a conversion is started
  // after the conversion takes place, registers are locked
  // therefore change channel first, then clear trigger flag (since conversion cannot be started
  // when flag is set
  
  
  // switch to channel other channel before clearing ADC trigger flag
  // clear channels that need to be set

  // copy current channel
  //hb = ADMUX & CHANNEL_MASK ;
  hb = mux; // same as above statement

  // clear bits 0,1 of ADMUX register
  ADMUX &= ~(CHANNEL_MASK);
  
  // set channel bits that need to be set
  mux = (mux+1)%NUM_CHANNELS;

  // set channel bits in ADMUX
  ADMUX |= mux;
  
  // timer1 flag MUST BE cleared in software (since it's not cleared in HW)
  // timer1 flag triggers ADC conversion
  TIFR1 |= (1<<ICF1); 
    
  // increment index by 2 for circular buffer
  // low byte, and then high byte
  uint8_t next = (head+INC) % BUF_SIZE;
  
  // if full, don't write to circ. buffer
  if(next == tail)
    ; // buffer is full, so do nothing
  else
  { 
   hb = hb << 6; // left shift so channel # occupies highest two bits
            
    // insert with current 'head' value
    buffer[head] = ADCL;
  
    buffer[head+1] = ADCH;
    buffer[head+1] |= hb;

    // skip first three ADC conversions
    if(NUM_CHANNELS - firstTime > 0)
      firstTime += 1;
    else
    {
      // update 'head' with next value
      head = next;
    }
    // return no error since buffer is not full
    // full = 0;
  }
  
  
}
 
// set up ADC in auto-trigger mode with 
// ADC trigger = timer1 ICR1 (starts an ADC conversion)
 void initADC(void)
{
  // leave REFS bits0,1 as 0 since we are using an external voltage as the voltage reference
  // set voltage reference to 1.1 V internal voltage
  //ADMUX |= (1<<REFS0) | (1<<REFS1);

 // set voltage reference to 5 Volts (AVCC)
 // ADMUX |= (1<<REFS0);

  
  
  // enable trigger for auto-conversion mode
  // where an external source triggers an ADC conversion
  ADCSRA |= (1<<ADATE);

  // set clock prescaler to 1/128 --> ADC clock
  // is sys-clock/128 = 125 kHz (111)
  ADCSRA |= (1<<ADPS1)|(1<<ADPS2)|(1<<ADPS0);

  // enable ADC interrupt so FIFO buffer can be updated
  // after a conversion is completed
  // ADIF flag bit is set when a conversion is complete
  // and data registers are updated
  ADCSRA |= (1<<ADIE);

  // set trigger source equal to timer1 capture event (ICR1)
  ADCSRB |= (1<<ADTS2)|(1<<ADTS1)|(1<<ADTS0);
  
  // reading in a 10 bit result
  // left adjust, only read 8 highest bits, stored in ADCH
  // ADMUX |= (1<<ADLAR);
  
  // disable input buffer
  DIDR0 |= (1<<ADC0D);
  
  // enable ADC
  ADCSRA |= (1<<ADEN);

}

// set up sampling rate of ADC
void initTimer1(void)
{
// CTC mode with TOP = ICR1 (counter is cleared when TCNT1==ICR1)
TCCR1B |= (1<<WGM12)|(1<<WGM13);
    
 // freq = 1 Hz
 ICR1 = 15625;

 // freq = 50 Hz = 50 S/sec
 //ICR1 = 313;
 
 // enable interrupt for checking
//TIMSK1 |= (1<<OCIE1A);

 // divide sys-clock by 1024 s.t. 
 // rate at which count=TOP is reached = 16e6/1024 = 15625 Hz
 TCCR1B |= (1<<CS12)|(1<<CS10); 
}

void initUART(void)
{ 
  // 19.2k Baud
  UBRR0H = 0;                        
  UBRR0L = 51;
  
  /* 8 data bits, 1 stop bit */
  UCSR0B |= (1 << TXEN0) | (1 << RXEN0);
  UCSR0C |= (1 << UCSZ01) | (1 << UCSZ00);   
}


void transmitByte(uint8_t data)
{
  // loop or wait until bit is set
  while( !(UCSR0A & (1<<UDRE0)) )
    ;
  
  // loop_until_bit_is_set(UCSR0A,UDRE0);  
  UDR0 = data;
}


void printByte(uint8_t byte){
/* Converts a byte to a string of decimal text, sends it */
transmitByte('0'+ (byte/100)); /* Hundreds */
transmitByte('0'+ ((byte/10) % 10)); /* Tens */
transmitByte('0'+ (byte % 10)); /* Ones */
}

// print 16-bit integer in decimal format
void printWord(uint16_t x)
{
  uint8_t channel = 0;
  // x contains 10 bit value + channel # located at highest two bits
  uint8_t *ptr = (uint8_t*) &x;
  
  // points to high byte now
  ptr = ptr + 1;

  // get channel #
  channel = (*ptr);

  // located at highest two bits
  channel = channel >> 6;

  // remove channel # from x and presevere lowest 2 bits
  (*ptr) = (*ptr) & ( (1<<1) | (1<<0) );

printByte(channel);
transmitByte(':');
  
  /* print up to 1000s */
transmitByte('0'+ (x/1000)); /* Thousands */
transmitByte('0'+ (x % 1000) / 100); /* Hundreds */
transmitByte('0'+ ((x % 100) / 10)); /* Tens */
transmitByte('0'+ (x % 10)); /* Ones */

transmitByte('\n');
}

// print string
void printString(const char string[])
{
  while(*string != '\0')
  {
    transmitByte(*string);
    string++;
  }
  transmitByte('\n');
}


// pop 10 bit result from BUFFER to UART terminal
void read(uint16_t *value)
{
  uint8_t* ptr = (uint8_t*)value;

  // pop low byte
  *ptr = buffer[tail];

  // pop high byte
 *(ptr+1) = buffer[tail+1];

  // increment tail pointer
  tail = (tail+INC)%BUF_SIZE;  
}

/*
 * Enter main loop
 * when buffer is not empty
 * print out an ADC value on the terminal screen
 */

int main(void)
{

  uint16_t adc = 0;
  uint8_t lb, ub;

  // enable pull-up resistor on all ADC pins
  PORTC |= ( (1<<PC0) | (1<<PC1) | (1<<PC2) ); 

  initADC();
  initUART();
  initTimer1();
  sei();
  
  while(1)
  {
    if(tail != head)
    {
      read(&adc); 
      printWord(adc);
    }
    else if(tail == head - 1)
      printString("FULL!"); 
  }

  return(0);
}


