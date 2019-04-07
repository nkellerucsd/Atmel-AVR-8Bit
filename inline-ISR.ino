/*
 * Example program using AVR in-line assembly code.
 * Requires a button which triggers an ISR upon being pressed.
 * After the button is pressed, the timer counter is started and LED is turned on
 * and the ISR for the button press is turned off.
 * After the timer expires, the LED is turned off and the timer is turned off and
 * the ISR for the button press is turned on.
 * 
 */
 
// function using in-line assembly and interrupts
// set-up timer1 to toggle LED state

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#define ledPin 0
#define irPin 2

#define BYTE 256
#define SEC 15625

#define TOGGLE_MASK 1<<ledPin
#define STOP_TIMER ~( (1<<CS12) | (1<<CS10) )
#define STOP_INT0 ~(1<<INT0)

volatile uint8_t numPrssd = 0, send = 0, rx = 0;

// when button is pressed, increment value

ISR(INT0_vect, ISR_NAKED)
{

  /*
   * Push status register and r0,r30-r31 values on the stack
   * after a button press
   */
   
  asm(
  "push r0 \n"
  "in __tmp_reg__, 0x3F\n"
  "push r0 \n"
  "push r30 \n"
  "push r31 \n"
  : : );
  
  // disable INT0 interrupt
  // equivalent C-code below:
  //EIMSK &= ~(1<<INT0); //use IN/OUT 
  asm(
  "cbi %0, %1 \n"  // copy EIMSK to temp. reg
  :
  :"I" (_SFR_IO_ADDR(EIMSK)), "I" (INT0)
  );
  
  // clear INT0 Flag by setting it
  // not necessary since flag is cleared by hardware after ISR executes
  //EIFR |= (1<<INTF0);
  /*
  asm(
  "sbi %0, %1 \n"
  : :"I" (_SFR_IO_ADDR(EIFR)),"I" (INTF0)
  );
  */

  // clear flag bit  
  asm(
  "ldi r24, %1 \n" // mask to OR with
  "in __tmp_reg__, %0 \n"
  "or r24, __tmp_reg__ \n"
  "out %0, r24 \n"
  :
  :"I" (_SFR_IO_ADDR(EIFR)),"I" (1<<INTF0)
  );
  
  
  // clear timer counter
  asm(
  "sts %0, 0 \n"
  "sts %1, 0 \n"
  :
  :"M" (_SFR_MEM_ADDR(TCNT1L)),"M" (_SFR_MEM_ADDR(TCNT1H))
  );
  
  
  // start timer
  // TCCR1B |= (1<<CS12)|(1<<CS10);
  asm(
  "lds r24, %0 \n" // copy TCCR1B to temp. register
  "ori r24, %1 \n"    // set CS12 bit
  "sts %0, r24 \n" // load temp. register back into TCCR1B
  :
  :"M"(_SFR_MEM_ADDR(TCCR1B)), "M" ( (1<<CS10 | 1<<CS12) )
  );
    
  // turn on LED
  asm(
  "sbi %0, %1 \n"
  :
  :"I" (_SFR_IO_ADDR(PORTB)),"I"(ledPin)
  );

  /*
   * push status register and other register values
   * from stack back to registers upon ISR exit
   */
  asm(
  "pop r31 \n"
  "pop r30 \n"
  "pop r0 \n"
  "out 0x3F, __tmp_reg__ \n"
  "pop r0 \n"
  "reti \n"
  : : );
  
}


// initialize interrupt that triggers upon a button press
void initINT0(void)
{
  EICRA |= (1<<ISC01); // falling edge triggers an interrupt
  EIMSK |= (1<<INT0); // enable INT0
  
}


// timer1 interrupt
ISR(TIMER1_COMPA_vect, ISR_NAKED)
{
  asm(
  "push r0 \n"
  "in __tmp_reg__, 0x3F\n"
  "push r0 \n"
  "push r24 \n"
  "push r30 \n"
  "push r31 \n"
  : : );

  // stop timer
  //TCCR1B &= ~((1<<CS10) | (1<<CS12));
  
  asm(
  "ldi r24, %1 \n"
  "com r24 \n" // invert all bits
  "lds __tmp_reg__, %0 \n"
  "and r24, __tmp_reg__ \n"
  "sts %0, r24 \n"
  :
  :"M" (_SFR_MEM_ADDR(TCCR1B)), "M" ( (1<<CS10) | (1<<CS12) ) 
  );  
  
  // enable "send" for displaying button press on serial terminal
  asm(
  "ldi r24, 1 \n"
  "st Z, r24 \n"
  :
  :"z"(&send)
  );
  
  
  // increment number which is printed on serial terminal
  asm(
  "ld __tmp_reg__, Z \n"
  "inc __tmp_reg__ \n"
  "st Z, __tmp_reg__ \n"
  :
  :"z"(&numPrssd)
  );
  
  // turn off LED
  asm(
  "cbi %0, %1 \n"
  :
  :"I" (_SFR_IO_ADDR(PORTB)),"I"(ledPin)
  );  
  
  // enable ISR for another button press
  // EIMSK |= (1<<INT0); use IN/OUT
  asm(
  "sbi %0, %1 \n"
  :
  : "I" (_SFR_IO_ADDR(EIMSK)), "I" (INT0)
  );
  
  
  
  asm(
  "pop r31 \n"
  "pop r30 \n"
  "pop r24 \n"
  "pop r0 \n"
  "out 0x3F, __tmp_reg__ \n"
  "pop r0 \n"
  "reti \n"
  : : );

  
}


void initTimer1(void)
{
  // 16 bit timer1 in CTC mode (OCR1A = TOP)
  TCCR1B |= (1<<WGM12);
  
  // start clock
  //  TCCR1B |= (1<<CS12)|(1<<CS10);

  TIMSK1 |= (1<<OCIE1A); // enable output compare match A
  OCR1A = 55625; // about 1 second

}




int main(void)
{
  initINT0();
  initUART();
  initTimer1();
  
  // enable pull-up for INT0
  PORTD |= (1<<irPin);
  sei();
  
  // enable ledPin as output pin
  DDRB |= (1<<ledPin);
  
  while(1)
  { 
      // print # of timers button is pressed after timer counter ISR is executed
      if(send) {
      printByte(numPrssd);
      transmitByte('\n');
      send=0;
      }
  }


  return(0);
}




