/*
 * ----------------------------------------v0.1.0----------------------------------------
 *
 * Se crea el programa basado en "UHC_v0.1.2", se generan funciones de:
 *         
 *			+ Control de servomotores
 *			+ Control de motores a pasos
 *			+ Comunicaci�n v�a serie
 *
 *		El software espera la recepci�n de informaci�n desde el microcontrolador maestro y se
 * mueve a la posici�n indicada.
 *
 *		Este microcontrolador solo recibe la posici�n final absoluta, el c�lculo de pasos se
 * realiza en la funci�n dedicada para eso.
 *
 *		Al llegar a la posici�n da un aviso de haber llegado al maestro y se mantiene ahi hasta
 * que le llega una nueva posici�n a la cual desplazarse.
 *
 *		Otros datos a enviar son:
 *
 *			+ Estoy en HOME
 *			+ Gripper abierto
 *			+ Gripper cerrado 
 * --------------------------------------------------------------------------------------
 *
 * Motores-Stepper-Servo.c
 *
 * Los objetivos son los siguientes:
 *		
 *		+ El microcontrolador usado ser� un m328p. 
 *		- Se usar� PWM para el control de los servomotores.
 *				-A�N FALTA LA CONVERSI�N DE GRADOS A VALOR CONTADOR
 *		- El control de los motores a pasos ser�n la principal tarea del microcontrolador.
 *				-A�N FALTA DECIDIR SI SE USAR� UNA TABLA LUT O UN C�LCULO EN TIEMPO REAL PARA LA ACELERACI�N DE LOS MOTORES
 *		+ Se habilitar� la comunicaci�n por SERIE para recibir los valores de posici�n a los que debe de moverse el robot y 
 *        avisar cuando se ha llegado.
 *		- Se habilitar� una interrupci�n externa con la intenci�n de detener los motores a pasos cuando exista alguna 
 *        eventualidad en los contenedores.
 *				-INTERRUPCION DE EMERGENCIA A�N NO IMPLEMENTADA
 *
 *	Informaci�n sobre el pinout m328p:
 *
 *                                   _________________
 *                       RESET - PD6|1 *   \___/    28|
 *                    D0 - RXD - PD0|2              27|
 *                    D1 - TXD - PD1|3              26|
 *           D2 - INTERRUPCI�N - PD2|4              25|---(PC2 - SENG - GRIPPER - A2) //Sensor en descici�n
 *  D3 - DIRECCI�N EN X - DIRX - PD3|5              24|PC1 - SENY - HOME Y - A1
 *  D4 - DIRECCI�N EN Y - DIRY - PD4|6              23|PC0 - SENX - HOME X - A0
 *                               VCC|7              22|
 *                               GND|8              21|
 *                            XTAL 1|9              20|
 *                            XTAL 2|10             19|
 *       D5 - PASO EN X - PULX - PD5|11             18|
 *       D6 - PASO EN Y - PULY - PD6|12             17|
 *                                  |13             16|PB2 - OC1B - SERVO 2 - D10
 *                                  |14             15|PB1 - OC1A - SERVO 1 - D9
 *                                   �����������������
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
#include <stdlib.h>

#define SENX PC0
#define SENY PC1
#define DIRX PD3
#define DIRY PD4
#define PULX PD5
#define PULY PD6

volatile uint8_t p = 0;
volatile uint8_t buffer[12];
volatile uint8_t rxflag = 0;
volatile uint8_t dataflag = 0; 

																														//VELOCIDAD M�X = 700RPM	-->		5600 mm/min
const unsigned long accLUT[][2] = {{0,2577320},	{1,55432},	{3,8013},	{7,4132},	{14,2392},	{24,1502},	{40,1006},  //pasos minimos para acc			=	4669
									{62,708},	{92,521},	{183,172},	{289,147},	{412,128},	{551,112},	{707,100},	//pasos minimos para desacc			=	4669
									{880,90},	{1071,82},	{1277,76},	{1499,70},	{1736,66},	{1986,63},	{2260,57},	//pasos minimos para usar esta LUT	=	9338
									{2542,55},	{2831,54},	{3129,53},	{3432,52},	{3738,51},	{4047,50},	{4669,50}};

//Posible necesidad de una estructura gripper que contenga  a su vez dos estructuras servo o una modificacion a la actual.

// Estructura para servomotores
typedef struct Servos {
	uint8_t gripper_op;
	uint8_t gripper_cl;
	//unsigned char status; //enviar estructura en vez de solo valores y guardar el status del servo aqui:	'O' para abierto
	//																										'C' para cerrado
	} Servo;

// Estructura para Motores a pasos
typedef struct Steppers {
	int32_t a_pos;
	int32_t i_pos; //ES NECESARIA?
	int32_t f_pos;
	uint32_t delay_time;                                //ELIMINANDO LA ACELERACION EN 0 PASOS ES POSIBLE HACER ESTA VARIABLE uint16_t EN VEZ DE uint32_t
	} Stepper;

// Funciones avanzadas
void sendOK(unsigned char command, uint8_t status);

// Funciones para Motores a pasos
void init_stepper ();
void step (char axis, uint32_t dly);
void mov (char axis, Stepper *_stepper);
void home (Stepper *x, Stepper *y, uint8_t servo1, uint8_t servo2);

// Funciones para servomotores
void init_gripper (uint8_t servo1, uint8_t servo2);
void gripper (uint8_t gripper1_pos, uint8_t gripper2_pos);

//Funciones para comunicaci�n
void init_UART ();
void tx_char (unsigned char data);
void tx_string(char *bigdata);

//Funciones miscelaneas
void delay_ms(uint32_t count);
void delay_us(uint32_t count);
uint8_t readX();
uint8_t readY();
uint32_t f_pos();

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
	
	unsigned char command = 0;
	uint8_t action;
	
	cli();
	//LA INTERRUPCI�N SE DAR� CUANDO LA SE�AL PASE A BAJO
	EICRA = 0b00000010;	//HABILITADO PARA CAMBIOS DE 1 A 0
	EIMSK = 0b00000001; //PIN DE INTERRUPCI�N HABILITADO
	EIFR =	0b00000000; //BANDERA DE INTERRUPCI�N LISTA
	
	sei();
	
    while (1) 
    {
		if(rxflag)
		{
			
			if (dataflag == 1) // COMANDO LISTO
				command = buffer[0];
			
			if (dataflag == 2) // POSICION X LISTA
				xStepper.f_pos = f_pos(); // VIGILAR EL COMPORTAMIENTO DE ESTA VARIABLE
			
			if (dataflag == 3) // POSICION Y LISTA
			{
				yStepper.f_pos = f_pos(); // VIGILAR EL COMPORTAMIENTO DE ESTA VARIABLE
				action = 1;
			}
			
			if (action)
			{
				//HACER LA RUTINA QUE SE SOLICITE
				switch (command)
				{
					case 'U':
						mov('y', &yStepper);
						sendOK('U', 1);
						break;
					
					case 'W':
						mov('y', &yStepper);
						sendOK('W', 1);
						break;
					
					case 'L':
						mov('x', &xStepper);
						sendOK('L', 1);
						break;
					
					case 'R': //POSIBLEMENTE ESTE CASO NO SEA NECESARIO
						mov('x', &xStepper);
						sendOK('R', 1);
						break;
					
					case 'T':
						gripper(servo1.gripper_cl, servo2.gripper_cl);
						sendOK('T', 1);
						break;
					
					case 'D':
						gripper(servo1.gripper_cl, servo2.gripper_cl);
						sendOK('D', 1);
						break;
					
					case 'H':
						home(&xStepper, &yStepper, servo1.gripper_op, servo2.gripper_op);
						sendOK('H', 1);
						break;
					
				}
				action = 0;
			}
			rxflag = 0;
		}
    }
}
//FUNCIONES AVANZADAS------------------------------------------------------------------------------------------------------------

void sendOK(unsigned char command, uint8_t status) //Funci�n terminada, no probada
{
	// Esta funci�n reenvia el comando y un OK al microcontrolador maestro cuando ha alcanzado el punto deseado.
	tx_char(command);
	tx_char('\r');
	if (status == 1)
	tx_string("OK");
	else
	tx_string("NO");
	tx_char('\r');		
}

//FUNCIONES DE MOVIMIENTO--------------------------------------------------------------------------------------------------------

void init_stepper () //Funci�n terminada, no probada
{
	//Configuraci�n de los pines de entrada para los sensores homeX y homeY
	DDRC &= ~(1 << SENX) | ~(1 << SENY);
	PORTC = 0x00; //PROBABLE NECESIDAD DE RESISTENCIAS PULL UP
	
	//Configuraci�n de los pines de pulso y direcci�n para los motores a pasos.
	DDRD |= (1 << DIRX) | (1 << DIRY) | (1 << PULX) | (1 << PULY);
	PORTD = 0x00;
}

void step (char axis, uint32_t dly) //Funci�n terminada, no probada
{
	//Determinaci�n de eje y paso en esa direcci�n
	uint8_t pul = 0;
	if (axis == 'x')
		pul = PULX;
	else if (axis == 'y')
		pul = PULY;

//++++++++++++++++++++++++++++++++++++++++++++REVISAR QUE EL TIEMPO DE CONMUTACI�N SEA SUFICIENTE++++++++++++++++++++++++++++++++
	PORTD |= (1 << pul);
	delay_us(5);
	PORTD &= ~(1 << pul);
	delay_us(dly);
}

void mov (char axis, Stepper *_stepper) //,no probada
{
	//Manejo de distancia dada por informaci�n llegada desde comunicaci�n.
	//Convenci�n de direcci�n:	n = Negativo, en ambos ejes
	//							p = Positivo, en ambos ejes
	
	char dir = 'p';
	uint32_t pos = 0;
	uint8_t dirflag = 0;
	uint32_t aux = 0;
	
	if ((_stepper->f_pos - _stepper->a_pos) > 0)
	{
		dir = 'p';
		dirflag = 1;
		aux = _stepper->f_pos - _stepper->a_pos;
	}
	else if ((_stepper->a_pos - _stepper->f_pos) > 0)
	{
		dir = 'n';
		dirflag = -1;
		aux = _stepper->a_pos - _stepper->f_pos;
	}
	
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
	
	for (uint32_t i = 0; i < aux; i++)
	{
		// C�DIGO CORRESPONIDENTE AL MOVIMIENTO CON ACCELERACI�N //*****(EN DECISI�N EL USO DE LUT O C�LCULO EN TIEMPO REAL)*****
		
		if(i == accLUT[pos][0])
		{
			_stepper->delay_time = accLUT[pos][1];
			pos++;
		}
		//C�DIGO CORRESPONDIENTE AL MOVIMIENTO CON DESACELERECI�N, ENCONTRAR LA CONDICION CORRECTA PARA USAR ESTE CICLO COMO DESACELERADOR 
		else if (i ==  accLUT[pos - 1][0]/*Valor determinado por la tabla LUT*/)
		{
			_stepper->delay_time = accLUT[pos - 1][1];
			pos--;
		}
		
		step(axis, _stepper->delay_time);	//Se da un paso
		_stepper->a_pos += dirflag;			//Se actualiza la posici�n final
	}
	pos = 0;
}

void home (Stepper *x, Stepper *y, uint8_t servo1, uint8_t servo2) //Funci�n terminada, no probada
{
	//Funci�n para llegar a home por medio de sensores.
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
	x->i_pos = 0;
	x->f_pos = 0;
	
	y->a_pos = 0;
	y->i_pos = 0;
	y->f_pos = 0;	
}

void init_gripper (uint8_t servo1, uint8_t servo2) //Funci�n terminada, no probada
{
	//Habilitaci�n de PWM por el timer 1
	DDRB |= (1 << PB1) | (1 << PB2);
	TCCR1A = 0xA2;
	TCCR1B = 0x1B;
	TCCR1C = 0;
	ICR1 = 4999;
	OCR1A = servo1;
	OCR1B = servo2;
}

void gripper (uint8_t gripper1_pos, uint8_t gripper2_pos) //Funci�n terminada, no probada
{
	//++++++++++++++++++++++++++++++++ GENERAR CONVERSIONES DE GRADOS A VALOR CONTADOR ++++++++++++++++++++++++++++++++++++++++++
	OCR1A = gripper1_pos;
	OCR1B = gripper2_pos;
}


//FUNCIONES DE COMUNICACI�N------------------------------------------------------------------------------------------------------

void init_UART () //Funci�n terminada, no probada
{
	//Configuraci�n utilizada: habilitaci�n de RX y TX, habilitaci�n de interrupciones por recepci�n 
	// DATA STREAM: |	8	|	N	|	1	|	500Kbps	| Para otros valores referirse a la tabla del datasheet 
	cli();
	UCSR0B |= (1 << RXCIE0) | (1 << RXEN0) | (1 << TXEN0);
	UCSR0C |= (1 << UCSZ01) | (1 << UCSZ00);
	UBRR0 = 1;
	sei();
}

void tx_char (unsigned char data) //Funci�n terminada, no probada
{
	while (!(UCSR0A & (1 << UDRE0)));
	UDR0 = data;
}

void tx_string (char *bigdata) //Funci�n terminada, no probada
{
	while (*bigdata != 0x00)
	{
		tx_char(*bigdata);
		bigdata++;
	}
}

//FUNCI�N DE RECEPCI�N BASADA EN INTERRUPCI�N, BUSCAR EN LA ZONA DE INTERRUPCIONES PARA 


//FUNCIONES MISCELANEAS----------------------------------------------------------------------------------------------------------

void delay_ms (uint32_t count) //Funci�n terminada, no probada 
{
	while(count--) 
		_delay_ms(1);
}

void delay_us (uint32_t count) //Funci�n terminada, no probada
{
	while(count--) 
		_delay_us(1);
}

uint8_t readX () //Funci�n terminada, no probada
{
	//Lectura de sensor X, regresa		1: SENSOR ACTIVO - 5V
	//									0: SENSOR INACTIVO - 0V
	if((PINC & (1 << SENX)) == 0)
		return 0;
	else
		return 1;
}

uint8_t readY () //Funci�n terminada, no probada
{
	//Lectura de sensor Y, regresa		1: SENSOR ACTIVO - 5V
	//									0: SENSOR INACTIVO - 0V
	if((PINC & (1 << SENY)) == 0)
		return 0;
	else
		return 1;
}

uint32_t f_pos () //Funci�n terminada, no probada
{
	
	//Funci�n para convertir los datos que llegan en un n�mero de 32 bits, este numero es la posicion final del robot
	// y es a donde debe llegar, el dato ser� guardado en
	uint8_t i = 0;
	uint32_t value;
	char *ptr;
	char pos[12];
	pos[0] = 0; //INICIALIZACI�N EXPLICITA DE 0 COMO �NICO VALOR
	
	while (i != 0x00)
	{
		pos[i] = buffer[i]; //REVISAR CON ESPECIAL ATENCI�N CUANDO EL VALOR SEA 0
		i++;
	}
	value = strtoul(pos, &ptr, 10);
	return value;
}

//INTERRUPCIONES-----------------------------------------------------------------------------------------------------------------

ISR (INT0_vect) //, no probada
{
	//C�DIGO CORRESPONDIENTE A CASOS EXTREMOS, PROBABLEMENTE UNA PARADA SUAVE DE MOTOR.
}

ISR (USART_RX_vect)//Funci�n terminada, no probada
{
	buffer[p] = UDR0;
	if (buffer[p++] == '\r')
	{
		rxflag = 1;
		dataflag++;
		buffer[p-1] = 0x00;
		p = 0;
	}
}