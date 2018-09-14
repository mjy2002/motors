/*
 * Motores-Stepper-Servo.c
 *
 * La idea principal de este programa es convertirse en una libreria posteriormente.
 * Los objetivos son los siguientes:
 *		
 *		+ El microcontrolador usado ser� un m328p. 
 *		- Se usar� PWM para el control de los servomotores.
 *				-AUN FALTA LA CONVERSI�N DE GRADOS A VALOR CONTADOR
 *		- El control de los motores a pasos ser�n la principal tarea del microcontrolador.
 *				-
 *		- Se habilitar� la comunicaci�n por SPI para recibir los valores de posici�n a los que debe de moverse el robot y avisar cuando se ha llegado.
 *		- Se habilitar� una interrupci�n externa con la intenci�n de detener los motores a pasos cuando exista alguna eventualidad en los contenedores.
 *
 * Created: 13/09/2018 08:40:47 a.m.
 * Author : arodriguez
 */ 

#define F_CPU 16000000UL
#include <avr/io.h>
#include <util/delay.h>

#define SENX PC0
#define SENY PC1
#define DIRX PD3
#define DIRY PD4
#define PULX PD5
#define PULY PD6

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
void mov (uint32_t pos, char axis, char dir);
void home (Stepper *x, Stepper *y);

// Funciones para servomotores
void init_gripper ();
void gripper (uint8_t gripper1_pos, uint8_t gripper2_pos);
/*
//Funciones para comunicaci�n
void init_slave_SPI ();
char recv_slave_SPI ();
void send_slave_SPI ();
*/

void delay_ms(uint32_t count);
void delay_us(uint32_t count);

int main (void)
{
	init_gripper();
	Servo servo1, servo2;
	servo1.gripper_op = 45;
	servo2.gripper_op = 135;
	servo1.gripper_cl = 30;
	servo2.gripper_cl = 150;
	
	init_stepper();
	Stepper xStepper, yStepper;
	
	home(&xStepper, &yStepper);
	_delay_ms(5000);
			
    while (1) 
    {
    }
}

void init_stepper () //Funci�n terminada, no probada
{
	//Configuraci�n de los pines de pulso y direcci�n para los motores a pasos.
	DDRD |= (1 << DIRX) | (1 << DIRY) | (1 << PULX) | (1 << PULY);
	PORTD = 0x00;
	
	//Configuraci�n de los pines de entrada para los sensores homeX y homeY
	DDRC &= ~(1 << SENX) | ~(1 << SENY);
	PORTC = 0x00; //PROBABLE NECESIDAD DE RESISTENCIAS PULL UP
}

void step (char axis, uint32_t dly) //Funci�n terminada, no probada
{
	//Determinaci�n de eje y paso en esa direcci�n
	uint8_t pul = 0;
	if (axis == 'x')
		pul = PULX;
	else if (axis == 'y')
		pul = PULY;

//++++++++++++++++++++++++++++++++++++++++++++REVISAR QUE EL TIEMPO DE CONMUTACI�N SEA SUFICIENTE+++++++++++++++++++++++++++++++++++++++
	PORTD |= (1 << pul);
	delay_us(5);
	PORTD &= ~(1 << pul);
	delay_us(dly); 
}

void mov (uint32_t pos, char axis, char dir)//,no probada
{
	//Manejo de distancia dada por informaci�n llegada desde comunicaci�n.
	//Convenci�n de direcci�n:	n = Negativo, en ambos ejes
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
	for (uint32_t i = 0; i < pos; i++)
	{
		//C�DIGO CORRESPONIDENTE AL MOVIMIENTO CON ACCELERACI�N //*****(EN DESICI�N EL USO DE LUT O C�LCULO EN TIEMPO REAL)******
		step(axis, dly);
		
	}
}

void home (Stepper *x, Stepper *y)//,no probada
{
	
	//Funci�n para llegar a home por medio de sensores.

	x->a_pos = 0;
	y->a_pos = 0;	
}

void init_gripper () //Funci�n terminada, no probada
{
	//Habilitaci�n de PWM por el timer 1
	DDRB |= (1 << PB1) | (1 << PB2);
	TCCR1A = 0xA2;
	TCCR1B = 0x1B;
	TCCR1C = 0;
	ICR1 = 4999;
}

void gripper (uint8_t gripper1_pos, uint8_t gripper2_pos)//,no probada
{
	//++++++++++++++++++++++++++++++++ GENERAR CONVERSIONES DE GRADOS A VALOR CONTADOR ++++++++++++++++++++++++++++++++++++++++
	OCR1A = gripper1_pos;
	OCR1B = gripper2_pos;
}

void init_slave_SPI ()//,no probada
{
//+++++++++++++++++++++++++++++++++++++++++++PROBABLEMENTE SE CAMBIE A COMUNICACI�N I2C+++++++++++++++++++++++++++++++++++++++++++++++++++
	//Inicializaci�n de comunicaci�n por SPI
}

char recv_slave_SPI ()//,no probada
{
//+++++++++++++++++++++++++++++++++++++++++++PROBABLEMENTE SE CAMBIE A COMUNICACI�N I2C+++++++++++++++++++++++++++++++++++++++++++++++++++
	//Recepci�n de datos por SPI
	return 0;
}

void send_slave_SPI ()//,no probada
{
//+++++++++++++++++++++++++++++++++++++++++++PROBABLEMENTE SE CAMBIE A COMUNICACI�N I2C+++++++++++++++++++++++++++++++++++++++++++++++++++
	//Env�o de comunicaci�n por SPI
}

void delay_ms(uint32_t count) {
	while(count--) {
		_delay_ms(1);

	}
}

void delay_us(uint32_t count) {
	while(count--) {
		_delay_us(1);

	}
}