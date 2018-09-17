#ifndef I2C_H_
#define I2C_H_

void i2c_iniciar();      //función para iniciar el módulo TWI I2C AVR
void i2c_espera();       //función de espera
void i2c_inicia_com();   //función que inicia la comunicación I2C AVR
void i2c_detener();      //función que detiene la comunicación I2C AVR
void i2c_envia_dato(unsigned char );   //función para enviar o escribir
//datos en el esclavo
unsigned char i2c_recibe_dato_ack();   //función para recibir o leer datos del esclavo
//enviando el bit ACK si se quiere leer mas datos
//después del último leído
unsigned char i2c_recibe_dato_nack();  //función para recibir o leer datos del esclavo
//sin enviar el bit ACK si no se quiere leer mas datos
//después del último leido
uint8_t i2c_estado_com();  //función para averiguar el estado de la comunicación I2C AVR
//útil para detectar errores

//inicialización del módulo TWI I2C AVR en el ATMEL STUDIO en una función////
//para el ATMEGA88 como maestro
//a 400KHz con un oscilador de 8Mhz

void i2c_iniciar()
{
	PORTC|=((1<<4)|(1<<5));  //activa resistencias pull upp para SCL y SDA
	TWBR=2;                  //velocidad 400Khz, Fosc 8Mhz, prescaler de 1
	TWCR|=(1<<TWEN);         //módulo TWI iniciado
}

// Función de espera: mientras el bit7 o bit TWINT del registro
// TWCR sea 0, el IC2 AVR se esperará
// antes de realizar algún trabajo

void i2c_espera()
{
	while ((TWCR & (1<<TWINT)) == 0);//espera mientras el  bit de interrupcion sea 0
}

// Función de inicio de la comunicación I2C AVR

void i2c_inicia_com() 
{
	TWCR = (1<<TWINT)|(1<<TWSTA)|(1<<TWEN);//bandera de interrupción a 1, start, habilita I2C AVR
	i2c_espera();       //espera mientras el bit TWINT sea 0
}

// Función de parada de la comunicación I2C I2C

void i2c_detener() 
{
	TWCR = (1<<TWINT)|(1<<TWSTO)|(1<<TWEN);//bandera de interrupción a 1, detener, habilita I2C AVR
}

//Función de transmisión de datos del maestro al esclavo

void i2c_envia_dato(unsigned char dato)
 {
	TWDR = dato;
	TWCR = (1<<TWINT)|(1<<TWEN);//para empezar a enviar el dato
	i2c_espera();//cuando TWINT se ponga a 1 se habrá terminado de enviar el dato
}

//Función de recepción de datos enviados por el esclavo al maestro
//esta función es para leer los datos que están en el esclavo
//en forma continua, esto es tras leer uno se volverá a leer otro

unsigned char i2c_recibe_dato_ack(){//maestro envia ack para seguir recibiendo
	//mas  datos
	TWCR = (1<<TWINT)|(1<<TWEN)|(1<<TWEA);
	i2c_espera();
	return TWDR;
}

//Función de recepción de datos enviados por el esclavo al maestro
//esta función es para leer solo un dato desde el esclavo
//esto es tras leer uno ya no se volverá a leer otro

unsigned char i2c_recibe_dato_nack(){//maestro no envia ack para no seguir recibiendo
	//mas  datos
	TWCR = (1<<TWINT)|(1<<TWEN);
	i2c_espera();
	return TWDR;
}

//función para averiguar el estado de la comunicación I2C AVR
//útil para detectar errores, el valor que retorna esta función
//se compara con el estado que deberían indicar los bits del 7 al 3
//del registro TWSR según tabla, durante la comunicación I2C AVR,

uint8_t i2c_estado_com(){
	uint8_t estado;        //variable donde se almacena el estado de la comunicación
	//I2C AVR
	estado = TWSR & 0xf8;  //en la variable estado se guarda el valor de los 5 bits de
	//mas peso del registro TWSR seguidos de 3 ceros,
	//el número obtenido indica
	//el estado en que se encuentra la comunicación I2C AVR
	
	return estado;         //la función retorna el estado de la comunicación
}

#endif /* I2C_H_ */