#ifndef I2C_H_
#define I2C_H_

void i2c_iniciar();      //funci�n para iniciar el m�dulo TWI I2C AVR
void i2c_espera();       //funci�n de espera
void i2c_inicia_com();   //funci�n que inicia la comunicaci�n I2C AVR
void i2c_detener();      //funci�n que detiene la comunicaci�n I2C AVR
void i2c_envia_dato(unsigned char );   //funci�n para enviar o escribir
//datos en el esclavo
unsigned char i2c_recibe_dato_ack();   //funci�n para recibir o leer datos del esclavo
//enviando el bit ACK si se quiere leer mas datos
//despu�s del �ltimo le�do
unsigned char i2c_recibe_dato_nack();  //funci�n para recibir o leer datos del esclavo
//sin enviar el bit ACK si no se quiere leer mas datos
//despu�s del �ltimo leido
uint8_t i2c_estado_com();  //funci�n para averiguar el estado de la comunicaci�n I2C AVR
//�til para detectar errores

//inicializaci�n del m�dulo TWI I2C AVR en el ATMEL STUDIO en una funci�n////
//para el ATMEGA88 como maestro
//a 400KHz con un oscilador de 8Mhz

void i2c_iniciar()
{
	PORTC|=((1<<4)|(1<<5));  //activa resistencias pull upp para SCL y SDA
	TWBR=2;                  //velocidad 400Khz, Fosc 8Mhz, prescaler de 1
	TWCR|=(1<<TWEN);         //m�dulo TWI iniciado
}

// Funci�n de espera: mientras el bit7 o bit TWINT del registro
// TWCR sea 0, el IC2 AVR se esperar�
// antes de realizar alg�n trabajo

void i2c_espera()
{
	while ((TWCR & (1<<TWINT)) == 0);//espera mientras el  bit de interrupcion sea 0
}

// Funci�n de inicio de la comunicaci�n I2C AVR

void i2c_inicia_com() 
{
	TWCR = (1<<TWINT)|(1<<TWSTA)|(1<<TWEN);//bandera de interrupci�n a 1, start, habilita I2C AVR
	i2c_espera();       //espera mientras el bit TWINT sea 0
}

// Funci�n de parada de la comunicaci�n I2C I2C

void i2c_detener() 
{
	TWCR = (1<<TWINT)|(1<<TWSTO)|(1<<TWEN);//bandera de interrupci�n a 1, detener, habilita I2C AVR
}

//Funci�n de transmisi�n de datos del maestro al esclavo

void i2c_envia_dato(unsigned char dato)
 {
	TWDR = dato;
	TWCR = (1<<TWINT)|(1<<TWEN);//para empezar a enviar el dato
	i2c_espera();//cuando TWINT se ponga a 1 se habr� terminado de enviar el dato
}

//Funci�n de recepci�n de datos enviados por el esclavo al maestro
//esta funci�n es para leer los datos que est�n en el esclavo
//en forma continua, esto es tras leer uno se volver� a leer otro

unsigned char i2c_recibe_dato_ack(){//maestro envia ack para seguir recibiendo
	//mas  datos
	TWCR = (1<<TWINT)|(1<<TWEN)|(1<<TWEA);
	i2c_espera();
	return TWDR;
}

//Funci�n de recepci�n de datos enviados por el esclavo al maestro
//esta funci�n es para leer solo un dato desde el esclavo
//esto es tras leer uno ya no se volver� a leer otro

unsigned char i2c_recibe_dato_nack(){//maestro no envia ack para no seguir recibiendo
	//mas  datos
	TWCR = (1<<TWINT)|(1<<TWEN);
	i2c_espera();
	return TWDR;
}

//funci�n para averiguar el estado de la comunicaci�n I2C AVR
//�til para detectar errores, el valor que retorna esta funci�n
//se compara con el estado que deber�an indicar los bits del 7 al 3
//del registro TWSR seg�n tabla, durante la comunicaci�n I2C AVR,

uint8_t i2c_estado_com(){
	uint8_t estado;        //variable donde se almacena el estado de la comunicaci�n
	//I2C AVR
	estado = TWSR & 0xf8;  //en la variable estado se guarda el valor de los 5 bits de
	//mas peso del registro TWSR seguidos de 3 ceros,
	//el n�mero obtenido indica
	//el estado en que se encuentra la comunicaci�n I2C AVR
	
	return estado;         //la funci�n retorna el estado de la comunicaci�n
}

#endif /* I2C_H_ */