/*
 * Mejora Serial 
 * 
 * Transmisor
 * 
 *  1. El transmisor envia los datos al receptor
 *    a. Al finalizar el envio el transmisor activa un time-out para evitar el error de que los datos no hayan llegado
 *    b. Si el time-out se agota se reenvian los datos
 *  
 *  2. El transmisor recibe el dato
 *    a. Se compara el dato que llego con el enviado previamente
 *        Caso 2a. Los datos son iguales, se procede a enviar un OK
 *        Caso 2b. Los datos son diferentes, se procede a enviar un NO
 *  
 *  3.  Caso 2a 
 *  
 *    1. Se espera la confirmacion del receptor, al recibirla se puede continuar con la ejecucion del programa.
 *    
 *      Caso 2b
 *      
 *     2. Se reinicia el proceso desde el punto 1 
 * 
 *  4. En caso de que se reciban mal los datos 10 veces se considera como una falla.
 *  
 *  
 * Receptor
 * 
 *  1. Se reciben los datos del transmisor
 *    a. Se considera fin de transmision el caracter de control, ya sea <ETX> o '\r'
 *  
 *  3.  Caso 2a
 *      
 *      1. Si el receptor recibe un OK:
 *        a. Se guarda el dato en la variable correspondiente
 *        b. Se envia un OK  de confirmación y se continua con la ejecución
 *      
 *      Caso 2b
 *      
 *      1. Si el receptor recibe un NO se considera una falla y se vuelve al punto 1.
 *      
 */

// TRANSMISION
/*
uint8_t envio(unsigned char data)
{
  uint8_t timeout = 0;
  tx_send(data);
  //SE ACTIVA TIMEOUT
  while(rxflag == 0 || timeout == 10);
  {
    if(//bandera del timer)
    {
      timeout++;
      tx_send(data);
      // Reinicia contador
    }
  }
  //Desactiva timer
  timeout = 0;
  rxflag = 0;
  
  if( data == // lo que haya en el buffer)
  {
    return 1;
  }
  else 
  {
    return 0;
  }
}
*/








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
  while(1)
  {
    if(rxflag == 1)
    {
      if(dataflag == 1)
      {
        _c = buffer[0];
        tx_char(_c);
        tx_char('\r');
        //tx_char('\n');
        tempflag = 1;
        while(tempflag == 1);
        _d = buffer[0];
        if(_d == 'O')
        {
          command = _c;
          //tx_string("CORRECTO\n");
          PORTB &= ~(1 << PB5); 
        }
        else
        {
          dataflag = 0;
          //tx_string("INCORRECTO\n");
        }
      }
      if(dataflag == 2)
      {
        //tx_string("dataflag = 2 \n");
        u = 0;
        memset(temp, 0, sizeof(temp));
        while(buffer[u] != 0x00)
        {
          temp[u] = buffer[u];
          u++;
        }
        value = strtol(temp, &ptr, 10);
        tx_string(temp);
        tx_char('\r');
        //tx_char('\n');
        tempflag = 1;
        while(tempflag == 1);
        _d = buffer[0];
        if(_d == 'O')
        {
          x = value;
          //tx_string("CORRECTO\n"); 
        }
        else
        {
          dataflag = 1;
          //tx_string("INCORRECTO\n");
        }
      }
      if(dataflag == 3)
      {
        //tx_string("dataflag = 3 \n");
        u = 0;
        memset(temp, 0, sizeof(temp));
        while(buffer[u] != 0x00)
        {
          temp[u] = buffer[u];
          u++;
        }
        value = strtol(temp, &ptr, 10);
        tx_string(temp);
        tx_char('\r');
        //tx_char('\n');
        tempflag = 1;
        while(tempflag == 1);
        _d = buffer[0];
        if(_d == 'O')
        {
          y = value;
          //tx_string("CORRECTO\n"); 
        }
        else
        {
          dataflag = 2;
          //tx_string("INCORRECTO\n");
        }
      }
      if(dataflag == 4)
      {
        _c = buffer[0];
        tx_char(_c);
        tx_char('\r');
        //tx_char('\n');
        tempflag = 1;
        while(tempflag == 1);
        _d = buffer[0];
        if(_d == 'O')
        {
          gripper = _c;
          dataflag = 0;
          PORTB |= (1 << PB5);
         /*
          tx_string("CORRECTO\n");
          tx_string("\nComando = ");
          tx_char(command);
          tx_string("\nx = ");
          ltoa(x, _buffer, 10);
          tx_string(_buffer);
          tx_string("\ny = ");
          ltoa(y, _buffer, 10);
          tx_string(_buffer);
          tx_string("\nGripper =  ");
          tx_char(gripper);
          tx_char('\n'); 
          */
        }
        else
        {
          dataflag = 3;
          //tx_string("INCORRECTO\n");
        }
      }
      rxflag = 0;
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
    }
    */
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

ISR (USART_RX_vect)
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
