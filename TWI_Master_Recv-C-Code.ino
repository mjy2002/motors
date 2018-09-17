#define F_CPU 16000000UL

#include <avr/io.h>
#include <util/twi.h>
#include <util/delay.h>


void init_TWI();
void wait_TWI();
void start_comm_TWI();
void stop_TWI();
void send_TWI(unsigned char data);

unsigned char recv_TWI_ack();
unsigned char recv_TWI_nack();



int main(void)
{
  DDRB |= (1 << PB7);
  PORTB = 0x00;
  unsigned char recv[3];
  int i = 0;
  init_TWI();
  
    while (1) 
    {
    start_comm_TWI();
    send_TWI(0x11);
    for(i = 0; i <= 2; i++)
      recv[i] = recv_TWI_ack();
    recv[3] = recv_TWI_nack();
    stop_TWI();
    
    if(recv[0] == 'h')
    {
      if(recv[1] == 'o')
      {
        if(recv[2] == 'l')
        {
          if(recv[3] == 'a')
          {
            PORTB ^= (1 << PB7);     
          }
        }
      }
    }
    _delay_ms(1000);
    recv[0] = 0;
    recv[1]= 0;
    recv[2] = 0;
    recv[3] = 0;
    }
}

void init_TWI()
{
  PORTC |= (1 << PC4) | (1 << PC5);   // Se habilitan las resistencias Pull Up internas
  TWBR = 72;                // Se configura el reloj del TWI a 100KHZ, para 400 KHZ usar 12
  TWCR |= (1 << TWEN);          //Se habilita el uso de comunicación TWI
}

void wait_TWI()
{
  while ((TWCR & (1 << TWINT)) == 0);   //Mantiene el programa congelado mientras se realiza la comunicación
}

void start_comm_TWI()
{
  TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN); //Pone el bit de interrupción en 1, activa el bit de start y habilita la comunicación 
  wait_TWI();
}

void stop_TWI()
{
  TWCR = (1 << TWINT) | (1 << TWSTO) | (1 << TWEN); //Pone el bit de interrupción en 1, activa el bit de start y habilita la comunicación
}

void send_TWI(unsigned char data)
{
  TWDR = data;                  //Se carga el dato a enviar en el registro del TWI
  TWCR = (1 << TWINT) | (1 << TWEN);        //Se pone el bit de interrupción en 1 y se habilita la comunicación
  wait_TWI();                   //Se espera a realizar el envío
}

unsigned char recv_TWI_nack()
{
  TWCR = (1 << TWINT) | (1 << TWEN);        //Se pone el bit de interrupción en 1 y se habilita la comunicación
  wait_TWI();                   //Se espera a realizar la recepción
  return TWDR;                  //Se regresa el dato recibido
}

unsigned char recv_TWI_ack()
{
  TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWEA);  //Se pone el bit de interrupción a 1, se habilita la comunicación y se envia el pulso de acknowledge
  wait_TWI();                     //Se espera a que la recepción sea terminada
  return TWDR;                    //Se regresa el dato recibido
}

