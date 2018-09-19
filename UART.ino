#define F_CPU 16000000UL
#include <avr/io.h>
#include <util/delay.h>

int main(void)
{
    unsigned char x[10];
    init_UART();
    uint16_t i = 0;
    DDRB |= (1 << PB5);
    PORTB &= ~(1 << PB5);
    while (1) 
    {
      itoa(i,x,10);
      tx_string(x);
      tx_char('\r'); 
      _delay_ms(100);
      i++;
    }
}

void init_UART ()
{
  UCSR0B = 0b00011000;
  UCSR0C = 0b00000110;
  UBRR0 = 1;
}

unsigned char rx_char ()
{
  if(UCSR0A & (1 << RXC0))
    return UDR0;
}

void tx_char (unsigned char data)
{
  while(!(UCSR0A & (1<<UDRE0)));
  UDR0 = data;
}

void tx_string (char *bigdata)
{
  while (*bigdata != 0x00)
  {
    tx_char (*bigdata);
    bigdata++;
  } 
}

