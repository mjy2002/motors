/*
 * Motores-Stepper-Servo.c
 *
 * La idea principal de este programa es convertirse en una libreria posteriormente.
 * Los objetivos son los siguientes:
 *		
 *		+ El microcontrolador usado será un m328p. 
 *		- Se usará PWM para el control de los servomotores.
 *				-AUN FALTA LA CONVERSIÓN DE GRADOS A VALOR CONTADOR
 *		- El control de los motores a pasos serán la principal tarea del microcontrolador.
 *				-
 *		- Se habilitará la comunicación por SERIE para recibir los valores de posición a los que debe de moverse el robot y avisar cuando se ha llegado.
 *		- Se habilitará una interrupción externa con la intención de detener los motores a pasos cuando exista alguna eventualidad en los contenedores.
 *
 *	Información sobre el pinout m328p:
 *
 *                                   _________________
 *                       RESET - PD6|1 *   \___/    28|PC5 - SCL - COMUNICACIÓN - A5
 *                                  |2              27|PC4 - SDA - COMUNICACIÓN - A4
 *                                  |3              26|
 *           D2 - INTERRUPCIÓN - PD2|4              25|
 *  D3 - DIRECCIÓN EN X - DIRX - PD3|5              24|PC1 - SENY - HOME Y - A1
 *  D4 - DIRECCIÓN EN Y - DIRY - PD4|6              23|PC0 - SENX - HOME X - A0
 *                                  |7              22|
 *                                  |8              21|
 *                                  |9              20|
 *                                  |10             19|
 *       D5 - PASO EN X - PULX - PD5|11             18|
 *       D6 - PASO EN Y - PULY - PD6|12             17|
 *                                  |13             16|PB2 - OC1B - SERVO 2 - D10
 *                                  |14             15|PB1 - OC1A - SERVO 1 - D9
 *                                   ¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯
 *								
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 * Created: 13/09/2018 08:40:47 a.m.
 * Author : arodriguez
 */ 

#define F_CPU 16000000UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#define SENX PC0
#define SENY PC1
#define DIRX PD3
#define DIRY PD4
#define PULX PD5
#define PULY PD6

volatile uint8_t p = 0;
volatile uint8_t buffer[20];
volatile uint8_t rxflag; 

// Estructura para servomotores
typedef struct Servos {
	uint8_t gripper_op;
	uint8_t gripper_cl;
	} Servo;

// Estructura para Motores a pasos
typedef struct Steppers {
	int32_t a_pos;
	int32_t f_pos;
	uint32_t delay_time;
	} Stepper;

// Funciones para motor a pasos
void init_stepper ();
void step (char axis, uint32_t dly);
void mov (uint32_t pos, char axis, char dir, Stepper *stpr);
void home (Stepper *x, Stepper *y, uint8_t servo1, uint8_t servo2);

// Funciones para servomotores
void init_gripper (uint8_t servo1, uint8_t servo2);
void gripper (uint8_t gripper1_pos, uint8_t gripper2_pos);

//Funciones para comunicación
void init_UART ();
void tx_char (unsigned char data);
void tx_string(char *bigdata);

//Funciones miscelaneas
void delay_ms(uint32_t count);
void delay_us(uint32_t count);
uint8_t readX();
uint8_t readY();

int main (void) // ,no probada
{
	init_UART();
	
	Servo servo1, servo2;
	servo1.gripper_op = 45;
	servo2.gripper_op = 135;
	servo1.gripper_cl = 30;
	servo2.gripper_cl = 150;
	init_gripper(servo1.gripper_op, servo2.gripper_op);
	
	Stepper xStepper, yStepper;
	init_stepper();
	home(&xStepper, &yStepper, servo1.gripper_op, servo2.gripper_op);
	delay_ms(5000);
	
	cli();
	//CÓDIGO DE HABILITACION PARA LA INTERRUPCIÓN
	sei();
	
    while (1) 
    {
		//Wait for a command
    }
}
//FUNCIONES AVANZADAS------------------------------------------------------------------------------------------------------------



//FUNCIONES DE MOVIMIENTO--------------------------------------------------------------------------------------------------------

void init_stepper () //Función terminada, no probada
{
	//Configuración de los pines de entrada para los sensores homeX y homeY
	DDRC &= ~(1 << SENX) | ~(1 << SENY);
	PORTC = 0x00; //PROBABLE NECESIDAD DE RESISTENCIAS PULL UP
	
	//Configuración de los pines de pulso y dirección para los motores a pasos.
	DDRD |= (1 << DIRX) | (1 << DIRY) | (1 << PULX) | (1 << PULY);
	PORTD = 0x00;
}

void step (char axis, uint32_t dly) //Función terminada, no probada
{
	//Determinación de eje y paso en esa dirección
	uint8_t pul = 0;
	if (axis == 'x')
		pul = PULX;
	else if (axis == 'y')
		pul = PULY;

//++++++++++++++++++++++++++++++++++++++++++++REVISAR QUE EL TIEMPO DE CONMUTACIÓN SEA SUFICIENTE++++++++++++++++++++++++++++++++
	PORTD |= (1 << pul);
	delay_us(5);
	PORTD &= ~(1 << pul);
	delay_us(dly);
}

void mov (uint32_t pos, char axis, char dir, Stepper *stpr) //,no probada
{
	//Manejo de distancia dada por información llegada desde comunicación.
	//Convención de dirección:	n = Negativo, en ambos ejes
	//							p = Positivo, en ambos ejes
	uint32_t dly = 0;
	if (dir == 'p')
	{
		if (axis == 'x')
		{
			PORTD |= (1 << DIRX);		
		}
		else if (axis == 'y')
		{
			PORTD &= ~(1 << DIRY);
		}	
	}
	else if(dir == 'n')
	{
		if (axis == 'x')
		{
			PORTD &= ~(1 << DIRX);
		}
		else if (axis == 'y')
		{
			PORTD |= (1 << DIRY);
		}
	}
	for (uint32_t i = 0; i < stpr->f_pos; i++)
	{
		// CÓDIGO CORRESPONIDENTE AL MOVIMIENTO CON ACCELERACIÓN //*****(EN DECISIÓN EL USO DE LUT O CÁLCULO EN TIEMPO REAL)*****
		
		
		step(axis, dly);
		
		
		// PASAR POR REFERENCIA EL MOTOR A PASOS A MOVER, PARA PODER ACTUALIZAR EL VALOR DE POSICION ACTUAL Y AHI MISMO CARGAR EL
		// VALOR DE POSICION FINAL PARA ELIMINAR uint32_t pos, PUES EL MISMO TRABAJO SE REALIZA EN LA ESTRUCTURA DEL STEPPER CON 
		// EL ATRIBUTO f_pos
	}
}

void home (Stepper *x, Stepper *y, uint8_t servo1, uint8_t servo2) //Función terminada, no probada
{
	//Función para llegar a home por medio de sensores.
	gripper(servo1, servo2);
	if(readX() == 0)
	{
		PORTD &= ~(1 << DIRX);
		while (readX() == 0)
			step('x',150);
	}
	if(readY() == 0)
	{
		PORTD |= (1 << DIRY);
		while (readX() == 0)
			step('y',150);
	}
	x->a_pos = 0;
	y->a_pos = 0;	
}

void init_gripper (uint8_t servo1, uint8_t servo2) //Función terminada, no probada
{
	//Habilitación de PWM por el timer 1
	DDRB |= (1 << PB1) | (1 << PB2);
	TCCR1A = 0xA2;
	TCCR1B = 0x1B;
	TCCR1C = 0;
	ICR1 = 4999;
	OCR1A = servo1;
	OCR1B = servo2;
}

void gripper (uint8_t gripper1_pos, uint8_t gripper2_pos) //Función terminada, no probada
{
	//++++++++++++++++++++++++++++++++ GENERAR CONVERSIONES DE GRADOS A VALOR CONTADOR ++++++++++++++++++++++++++++++++++++++++++
	OCR1A = gripper1_pos;
	OCR1B = gripper2_pos;
}


//FUNCIONES DE COMUNICACIÓN------------------------------------------------------------------------------------------------------

void init_UART () //Función terminada, no probada
{
	//Configuración utilizada: habilitación de RX y TX, habilitación de interrupciones por recepción 
	// DATA STREAM: |	8	|	N	|	1	|	500Kbps	| Para otros valores referirse a la tabla del datasheet 
	cli();
	UCSR0B |= (1 << RXCIE0) | (1 << RXEN0) | (1 << TXEN0);
	UCSR0C |= (1 << UCSZ01) | (1 << UCSZ00);
	UBRR0 = 1;
	sei();
}

void tx_char (unsigned char data) //Función terminada, no probada
{
	while (!(UCSR0A & (1 << UDRE0)));
	UDR0 = data;
}

void tx_string(char *bigdata) //Función terminada, no probada
{
	while (*bigdata != 0x00)
	{
		tx_char(*bigdata);
		bigdata++;
	}
}


//FUNCIONES MISCELANEAS----------------------------------------------------------------------------------------------------------

void delay_ms (uint32_t count) //Función terminada, no probada 
{
	while(count--) 
		_delay_ms(1);
}

void delay_us (uint32_t count) //Función terminada, no probada
{
	while(count--) 
		_delay_us(1);
}

uint8_t readX() //Función terminada, no probada
{
	//Lectura de sensor X, regresa		1: SENSOR ACTIVO - 5V
	//									0: SENSOR INACTIVO - 0V
	if((PINC & (1 << SENX)) == 0)
		return 0;
	else
		return 1;
}

uint8_t readY() //Función terminada, no probada
{
	//Lectura de sensor Y, regresa		1: SENSOR ACTIVO - 5V
	//									0: SENSOR INACTIVO - 0V
	if((PINC & (1 << SENY)) == 0)
		return 0;
	else
		return 1;
}


//INTERRUPCIONES-----------------------------------------------------------------------------------------------------------------

ISR(INT0_vect) //, no probada
{
	
}

ISR(USART_RX_vect)//Función terminada, no probada
{
	buffer[p] = UDR0;
	if (buffer[p++] == '\r')
	{
		rxflag = 1;
		buffer[p-1] = 0x00;
		p = 0;
	}
}