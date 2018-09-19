#define F_CPU 16000000UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

volatile uint8_t i = 0;
volatile uint8_t _buffer[50];
volatile uint8_t rxflag = 0;

void init_interrupt_UART ();


int main(void)
{
  init_UART();
  init_interrupt_UART();
  
  while (1) 
  {
    if(rxflag)
    {
      tx_string(_buffer);
      tx_char('\r');
      tx_char('\n');
      rxflag = 0;
    }
  }
}

void init_UART ()
{
  UCSR0B |= (1 << RXEN0) | (1 << TXEN0);
  UCSR0C |= (1 << UCSZ01) | (1 << UCSZ00); 
  UBRR0 = 1;
}

void init_interrupt_UART ()
{
  cli();
  UCSR1B |= (1 << RXCIE1) | (1 << RXEN1) | (1 << TXEN1); //TRANSMISIÓN Y RECEPCIÓN HABILITADOS, INTERRUPCION POR RECEPCIÓN HABILITADA
  UCSR1C |= (1 << UCSZ11) | (1 << UCSZ10);               //CONFIGURACIÓN DEL FRAME 8Bit, No paridad, 1Bit Parada => 8N1
  UBRR1 = 1;                                            //CONFIGURACIÓN DE BAUDRATE A 500K
  sei();
}

ISR(USART1_RX_vect)
{
  _buffer[i] = UDR1;
  if(_buffer[i++] == '\r')
  {
    rxflag = 1;
    _buffer[i-1] = 0x00;
    i = 0;
  }
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
