#include <avr/io.h>
#include <avr/interrupt.h>

int speed = 0;
int sec1 = 0 ;
int sec2 = 0 ;
int min1 = 0 ;
int min2 = 0 ;
int hr1 = 0 ;
int hr2 = 0 ; 

void sev_seg(void)                                             // FUNCTION TO MANAGE THE CHANGE OF NUMBERS IN THE TIMER
{
  if (sec1 == 10 )
  {
    sec2++;
    sec1=0;
  }
  if(sec2 == 6)
  {
    min1++;
    sec2=0;
  }
  if(min1 == 10)
  {
    min2++;
    min1=0;
  }
  if (min2 == 6)
  {
    hr1++;
    min2=0;
  }
  if(hr1 == 10)
  {
    hr2++;
    hr1=0;
  }
}

ISR(INT2_vect)
{
  PORTB = PORTB ^ (1<<PB0) ;
  Timer1_CTC();
  PORTB = PORTB ^ (1<<PB4) ;
  TIMER0_PWM_init(10);
}
ISR(INT1_vect)                                                 // speed decrease ISR
{
  speed -=10;
  if(speed != 0 )
    {
      TIMER0_PWM_init(speed);
    }
  else if (speed <= 0)
      speed = 10;
}
ISR(INT0_vect)                                                // speed increase ISR
{
  speed+=10;
  if (speed != 100)
    {
      TIMER0_PWM_init(speed);
    }
  else if (speed >= 100)
    speed = 90;  
}

ISR( TIMER1_COMPA_vect)                                       // Timer ISR
{
  if (sec1 == 10)
  {
    sec1 = 0 ;
    PORTC = (PORTC & 0xF0)|(sec1 & 0x0F);
  }
  else
  {
    sec1++ ;
    PORTC = (PORTC & 0xF0)|(sec1 & 0x0F);
  }
  
}
void Timer1_CTC(void)
{
  TCNT1 = 0 ;                              // ITIAL VALUE OF THE TIMER
  TCCR1B = (1<<WGM12)|(1<<CS10)|(1<<CS12); // CONFIGURATION OF PRESCALAR AT 1024 
  OCR1A = 1000;                            // ENDING VALUE FOR THE COUNTER
  TIMSK |= (1<<4);                         // TO ENABLE THE INTERRUPT COMPARE MODE OF THE TIMER
  TCCR1A = (1<<FOC1A);                     // ENABLED IN NON PWM MODE
  SREG |= (1<<7);                          // ENABLE THE GENERAL INTERRUPT REG
}



void INT0_init()
{
  GICR |= (1<<6);
  MCUCR |= (1<<ISC01)|(1<<ISC01);
  SREG |= (1<<7);
}
void INT1_init()
{
  GICR  |= (1<<7);
  MCUCR |= (1<<ISC10)|(1<<ISC11);
  SREG  |= (1<<7);
}
void INT2_init()
{
  GICR  |= (1<<5);
  MCUCSR|= (1<<6); 
  SREG  |= (1<<7);
}
void TIMER0_PWM_init(char speed)
{
  char Output = ((speed*255)/100);
  TCCR0 = (1<<WGM00)|(1<<WGM01)|(1<<CS01)|(1<<COM01);            // FOR FAST PWM ACTIVATION , PRESCALER = 8 , NON_INVERING MODE
  OCR0 = Output;

}


int main(void)
{
  DDRB |= (1<<PB3) | (1<<PB0) | (1<<PB1) | (1<<PB4);
  DDRB = (DDRB & 0x0F)&(~(1 << PB5))&(~(1<< PB6));
  DDRD &= (!(1<<0))&(!(1<<1))&(!(1<<2))&(!(1<<3));
  DDRC = 0x0F;
  INT0_init();
  INT1_init();
  INT2_init();
  while(1)
  {
    if(PORTB & (1<<PB4))                                         // TIMER DISPLAY
    {
      PORTA = (1<<0);
      PORTC = (PORTC & 0xF0)|(hr2);
      _delay_ms(2);

      PORTA = (1<<1);
      PORTC = (PORTC & 0xF0)|(hr1);
      _delay_ms(2);

      PORTA = (1<<2);
      PORTC = (PORTC & 0xF0)|(min2);
      _delay_ms(2);

      PORTA = (1<<3);
      PORTC = (PORTC & 0xF0)|(min1);
      _delay_ms(2);

      PORTA = (1<<4);
      PORTC = (PORTC & 0xF0)|(sec2);
      _delay_ms(2);

      PORTA = (1<<5);
      PORTC = (PORTC & 0xF0)|(sec1);
      _delay_ms(2);
    }
    else
    {
      PORTA &= 0 ;
    }
    
    sev_seg();                                                     // SEVEN SEGMENT INCREAMENT FUNCTION

    if (PINB&(1<<5))                                               // PAUSE BUTTON 
    {
      PORTB = (PORTB & 0xF0) & (~(1<<0));
      TCCR1B = 0 ;
      speed = 10 ;
      TIMER0_PWM_init(speed);
    }
    else if (PINB&(1<<6))                                          // RESUME BUTTON
    {
      PORTB = (PORTB & 0xF0)|(1<<0);
      TCCR1B = (1<<WGM12)|(1<<CS10) | (1<<CS12);
    }
  }
}