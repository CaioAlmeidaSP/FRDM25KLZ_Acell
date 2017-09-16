/*
 * I2C.c
 *
 *  Created on: 24/08/2017
 *      Author: Caio
 */
#include "I2C.h"

void I2C0_int(void){
	// Inicializa o I2C0 nos pinos PTE24 - SCL e PTE25 - SDA

	//Habilitar Clock para Perifericos
	SIM_SCGC4 |= 1<<6; // Habilita clock para o periferico I2C0
	SIM_SCGC5 |= 1<<13;// Habilita clock para o periferico PortE

	//Configura pinos
	PORTE_PCR24 = 0x0500; //Habilita Alternative Function 5(I2C0_SCL) para o pino PTE24
	PORTE_PCR25 = 0x0500; //Habilita Alternative Function 5(I2C0_SDA) para o pino PTE25

	//Configura o periferico I2C0
	I2C0_C1 = 0x00;// Desabilita o I2C0 para configurar
	I2C0_S = 0x02;//Garante que o flag de interrupcao esteja limpo
	I2C0_F = 0x1F;//Configura o divisor de frequencia para valor de 240, com bus clock de 24MHz, teremos 100MHz
	I2C0_C1 = 0x80;//Habilita o modulo SPI
}

int I2C0_byteWrite(unsigned char slaveAddr, unsigned char memAddr, unsigned char data) {
	int timeout = 1000;

	while (I2C0_S & 0x20) { //Verifica se o I2C0 esta disponivel, Bus Busy
		if (--timeout <= 0)
			return ERR_BUS_BUSY;
		delay_us(100);
	}

	/* Inicio Envio*/

	I2C0_C1 |= 1<<4; //Habilita o I2C0 como master e para transmissão
	I2C0_C1 |= 1<<5;

	//Envia o Endereço do Slave e o Write Bit
	I2C0_D = slaveAddr << 1;//Assim que o dado é escrito no registrador o envio é iniciado
	while(!(I2C0_S & 0x02)); //Espera a transmissão do byte, verifica o Interrupt Flag
	I2C0_S |= 0x02; //Limpa o Interrupt Flag
	if (I2C0_S & 0x10) { // Verifica o flag arbitration lost (apenas em caso de multi master)
		I2C0_S |= 0x10; //Limpa o flag arbitration e retorna erro
		return ERR_ARB_LOST;
	}
	if (I2C0_S & 0x01) // Verifica se foi recebido o NACK do slave
		return ERR_NO_ACK;

	/* send memory address */
	I2C0_D = memAddr;
	while(!(I2C0_S & 0x02)); //Espera a transmissão do byte, verifica o Interrupt Flag
	I2C0_S |= 0x02; //Limpa o Interrupt Flag
	if (I2C0_S & 0x01) // Verifica se foi recebido o NACK do slave
		return ERR_NO_ACK;

	/* send data */
	I2C0_D = data;
	while(!(I2C0_S & 0x02)); //Espera a transmissão do byte, verifica o Interrupt Flag
	I2C0_S |= 0x02; //Limpa o Interrupt Flag
	if (I2C0_S & 0x01) // Verifica se foi recebido o NACK do slave
	return ERR_NO_ACK;

	/* stop */
	I2C0_C1 &= ~0x30;//Retorna o I2C0 para modo Slave e como receptor
	return ERR_NONE;//Retorna sem erro
}



int I2C0_byteRead(unsigned char slaveAddr, unsigned char memAddr, unsigned char* data) {
	int timeout = 100;
	volatile unsigned char lixo;
	while (I2C0_S & 0x20) { //Verifica se o I2C0 esta disponivel, Bus Busy
		if (--timeout <= 0)
			return ERR_BUS_BUSY;
		delay_us(100);
	}
	/* start */
	I2C0_C1 |= 1<<4;  //Habilita o I2C0 como transmissão
	I2C0_C1 |= 1<<5;  //Habilita o I2C0 como master

	/* Envia o Endereço do Slave e o Flag de Comando */
	I2C0_D = slaveAddr << 1;//
	while(!(I2C0_S & 0x02)); //Espera a transferencia ser completa
	I2C0_S |= 0x02; //Limpa o flag de transferencia completa
	if (I2C0_S & 0x10) //Verifica erro de arbitration lost
		return ERR_ARB_LOST;
	if (I2C0_S & 0x01) /* got NACK from slave */
		return ERR_NO_ACK;

	//Envia o endereço do registro a ser lido
	I2C0_D = memAddr;
	while(!(I2C0_S & 0x02)); /* wait for transfer complete */
	I2C0_S |= 0x02; /* clear IF */
	if (I2C0_S & 0x01) //Verifica se o sinal de NACK foi recebido do Slave
		return ERR_NO_ACK;//Retorna erro caso ACK recebido

	//Gerar o sinal de Reestart para o Slave
	I2C0_C1 |= 0x04; //Envia restart para o Slave

	/* Envia o Endereço do Slave e o Flag de Comando */
	I2C0_D = (slaveAddr << 1) | 1;//Envia o endereço com o flag para leitura
	while(!(I2C0_S & 0x02)); //Espera a transferencia ser completa
	I2C0_S |= 0x02; //Limpa o flag de transferencia completa
	if (I2C0_S & 0x01) //Verifica se o sinal de NACK foi recebido do Slave
		return ERR_NO_ACK;//Retorna erro caso ACK recebido

	//Altera o I2C para leitura
	I2C0_C1 &= ~0x18; //Habilita o modo RX e prepara para enviar ACK */
	I2C0_C1 |= 0x08; //Prepara para enviar NACK
	lixo = I2C0_D; //É necessário realizar uma leitura em modo Master para habilitar o recebimento do proximo byte
	while(!(I2C0_S & 0x02)); //Espera a transferencia ser completa
	I2C0_S |= 0x02; //Limpa o flag de transferencia
	I2C0_C1 |= 0x08; //Prepara para enviar NACK
	I2C0_C1 &= ~0x20; //Prepara para gerar o sinal de Stop antes de ler o ultimo byte
	*data = I2C0_D; //Lê o dado recebido
	return ERR_NONE;//Retorna 0 caso não ocorra nenhum erro
}


/* Use burst read to read multiple bytes from consecutive memory locations.
Burst read: S-(saddr+w)-ACK-maddr-ACK-R-(saddr+r)-data-ACK-data-ACK-…-data-
NACK-P
*/
int I2C0_MultiByteRead(unsigned char slaveAddr, unsigned char memAddr, int byteCount, unsigned char* data, int* cnt) {
	int timeout = 100;
	volatile unsigned char lixo;
	*cnt = 0;
	while (I2C0_S & 0x20) { //Verifica se o I2C0 esta disponivel, Bus Busy
		if (--timeout <= 0)
			return ERR_BUS_BUSY;
		delay_us(100);
	}
	/* start */
	I2C0_C1 |= 1<<4;  //Habilita o I2C0 como transmissão
	I2C0_C1 |= 1<<5;  //Habilita o I2C0 como master

	/* Envia o Endereço do Slave e o Flag de Comando */
	I2C0_D = slaveAddr << 1;//
	while(!(I2C0_S & 0x02)); //Espera a transferencia ser completa
	I2C0_S |= 0x02; //Limpa o flag de transferencia completa
	if (I2C0_S & 0x10) //Verifica erro de arbitration lost
		return ERR_ARB_LOST;
	if (I2C0_S & 0x01) /* got NACK from slave */
		return ERR_NO_ACK;

	/* send address of target register in slave */
	I2C0_D = memAddr;
	while(!(I2C0_S & 0x02)); /* wait for transfer complete */
	I2C0_S |= 0x02; /* clear IF */
	if (I2C0_S & 0x01) //Verifica se o sinal de NACK foi recebido do Slave
		return ERR_NO_ACK;//Retorna erro caso ACK recebido

	/* restart */
	I2C0_C1 |= 0x04; //Envia restart para o Slave

	/* send slave address and read flag */
	I2C0_D = (slaveAddr << 1) | 1;//Envia o endereço com o flag para leitura
	while(!(I2C0_S & 0x02)); //Espera a transferencia ser completa
	I2C0_S |= 0x02; //Limpa o flag de transferencia completa
	if (I2C0_S & 0x01) //Verifica se o sinal de NACK foi recebido do Slave
		return ERR_NO_ACK;//Retorna erro caso ACK recebido

	//Altera o I2C para leitura
	I2C0_C1 &= ~0x18; //Habilita o modo RX e prepara para enviar ACK */
	if (byteCount == 1)// Caso seja enviado apenas 1 byte
		I2C0_C1 |= 0x08; //Prepara para enviar NACK
	lixo = I2C0_D; //É necessário realizar uma leitura em modo Master para habilitar o recebimento do proximo byte

	//Leitura dos bytes
	while (byteCount > 0) {
		if (byteCount == 1)
			I2C0_C1 |= 0x08; //Prepara o NACK caso seja a ultima leitura
		while(!(I2C0_S & 0x02)); //Espera a transferencia ser completa
		I2C0_S |= 0x02; //Limpa o flag de transferencia
		if (byteCount == 1) {
			I2C0_C1 &= ~0x20; //Prepara para gerar o sinal de Stop antes de ler o ultimo byte
		}
		*data++ = I2C0_D; //Lê o dado recebido
		byteCount--;
		(*cnt)++;//Indica quantos bytes foram lido com sucesso
	}
	return ERR_NONE;//Retorna 0 caso não ocorra nenhum erro
}


