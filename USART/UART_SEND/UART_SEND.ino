#define F_CPU 16000000UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdlib.h>
#include <string.h>

volatile uint8_t p = 0;
volatile uint8_t buffer[12];
volatile uint8_t rxflag = 0;
volatile uint8_t dataflag = 0; 
volatile uint8_t tempflag = 0;

void init_UART ();
void tx_string (char *bigdata);
void tx_char (unsigned char data);
uint8_t tx_send(unsigned char c, int32_t xy);

int main (void)
{
  uint8_t f = 0;
  int32_t i = -707596;
  uint8_t j = 0;
  unsigned char  c = 'x';
  char datos[] = {'U', 'W', 'R', 'L', 'T', 'D', 'H'}; 
  init_UART();

  char _c = 0;
  char _d = 0;
  char command;

  uint8_t flag = 0;
  uint8_t u = 0;
  int32_t value = 20000001;
  char _buffer [sizeof(long)*8+1];
  char *ptr;
  char temp[12];
  uint32_t x = 0;
  uint32_t y = 0;
  char gripper = 0;

  DDRB |= (1 << PB5);
  PORTB |= (1 << PB5);
  _delay_ms(2500);
  while(1)
  {
    
    f= 0;
    f = tx_send('U', 10233);
    if(f == 1)
      u = 1;
    _delay_ms(10);
    f = 0;
    f = tx_send('x', 19243);
    if(f == 1)
      u = 2;
    _delay_ms(10);
    f = 0;
    f = tx_send('x', 10233);
    if(f == 1)
      u = 3;
    _delay_ms(10);
    f = 0;
    f = tx_send('O', 12343);
    if(f == 1)
      u = 4;
    if(u == 4)
    {
      for(u = 0; u < 10; u++)
      {
        PORTB ^= (1 << PB5);
        _delay_ms(250); 
      }
    }
    /*
    f = 0;
    f = tx_send(c,i);
    if(f == 1)
    {
      f = 0;
      c = datos[j];
      f = tx_send(c, i); 
      if(f == 1)
      {
        c = 'x';
        i++;
        j++; 
      }
      if(j == 7)
        j = 0;
    }*/
  }
}

void init_UART ()
{
  cli();
  UCSR0B |= (1 << RXCIE0) | (1 << RXEN0) | (1 << TXEN0);
  UCSR0C |= (1 << UCSZ01) | (1 << UCSZ00);
  UBRR0 = 1;
  sei();
}

void tx_char (unsigned char data)
{
  while (!(UCSR0A & (1 << UDRE0)));
  UDR0 = data;
}

void tx_string (char *bigdata)
{
  while (*bigdata != 0x00)
  {
    tx_char(*bigdata);
    bigdata++;
  }
}

ISR (USART0_RX_vect)
{
  buffer[p] = UDR0;
  if (buffer[p++] == '\r')
  {
    rxflag = 1;
    buffer[p-1] = 0x00;
    p = 0;
    if(tempflag == 0)
    {
      dataflag++;
    }
    tempflag = 0;
  }
}

uint8_t tx_send(unsigned char c, int32_t xy)
{
  uint8_t flag = 0;
  uint8_t i = 0;
  int32_t value = 20000001;
  char _buffer [sizeof(long)*8+1];
  char *ptr;
  char temp[12];
  char _c = 0;
  
  if(c == 'x')
  {
    do
    {
      ltoa(xy, _buffer, 10);
      tx_string(_buffer);
      tx_char('\r');
    
      while(rxflag == 0);
    
      if(rxflag == 1)
      {
        i = 0;
        memset(temp, 0, sizeof(temp));
        while(buffer[i] != 0x00)
        {
          temp[i] = buffer[i];
          i++;
        }
        value = strtol(temp, &ptr, 10);
        if(value == xy)
        {
          flag = 1; 
          tx_char( 'O');
          tx_char('\r');
        }
        else
        {
          flag = 0;
          tx_char( 'N');
          tx_char('\r');
        }
        rxflag = 0;
      }
    }while(flag == 0); 
  }
  else
  {
    do
    {
      tx_char(c);
      tx_char('\r');
      
      while(rxflag == 0);
        
      if(rxflag == 1)
      {
        _c = buffer[0];
        if(_c == c)
        {
          flag = 1; 
          tx_char( 'O');
          tx_char('\r');
        }
        else
        {
          flag = 0;
          tx_char( 'N');
          tx_char('\r');
        }
        rxflag = 0;
      }
    }while(flag == 0);
  }
  if(flag == 1)
    return 1;
  else
    return 0;
}
