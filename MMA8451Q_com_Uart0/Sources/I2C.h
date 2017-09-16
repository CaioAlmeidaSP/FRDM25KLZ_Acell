/*
 * I2C.h
 *
 *  Created on: 24/08/2017
 *      Author: Caio
 */

#ifndef SOURCES_I2C_H_
#define SOURCES_I2C_H_

#include "MKL25Z4.h"

#define ERR_NONE 0
#define ERR_NO_ACK 0x01
#define ERR_ARB_LOST 0x02
#define ERR_BUS_BUSY 0x03


//Fun��o para inicializar o I2C0 nos pinos PTE24 - SCL e PTE25 - SDA
void I2C0_int(void);

/*
 * Fun��o para escrever um unico byte no Slave
 * Retorna um valor Int com o codigo de erro ou valor 0 em caso de sucesso
 *slaveAddr(Byte)-> Endere�o de rede do Slave
 *memAddr(Byte)-> Endere�o de mem�ria do Slave em que ser� escrito
 *data(Byte)-> Valor a ser escrito no endere�o de mem�ria memAddr
 *Exemplo de uso: erro=I2C0_byteWrite(0x1D, 0x05, 0xFF)
*/
int I2C0_byteWrite(unsigned char slaveAddr, unsigned char memAddr, unsigned char data);


/*
 * Fun��o para ler um unico byte do Slave
 * Retorna um valor Int com o codigo de erro ou valor 0 em caso de sucesso
 *slaveAddr(Byte)-> Endere�o de rede do Slave
 *memAddr(Byte)-> Endere�o de mem�ria do Slave que ser� lido
 *data(Byte)-> Variavel em que ser� copiado o valor lido do endere�o de mem�ria memAddr
 *Exemplo de uso: erro=I2C0_byteRead(0x1D, 0x05, &variavel)
*/
int I2C0_byteRead(unsigned char slaveAddr, unsigned char memAddr, unsigned char* data);

/*
 * Fun��o para ler multiplos bytes do Slave
 * Retorna um valor Int com o codigo de erro ou valor 0 em caso de sucesso
 *slaveAddr(Byte)-> Endere�o de rede do Slave
 *memAddr(Byte)-> Endere�o de mem�ria do Slave que ser� iniciado as leituras
 *byteCount(Inteiro)->Numero de Bytes que ser�o lidos
 *data(array de Byte)-> Ponteiro para o array de bytes em que ser�o copiados os valores lidos do endere�o inicial de mem�ria memAddr
 *cnt(Inteiro)->Variavel em que ser� copiado o numero de Bytes lidos com sucesso
 *Exemplo de uso: erro=I2C0_MultiByteRead(0x1D, 0x05, 10, array, &contagem)
*/
int I2C0_MultiByteRead(unsigned char slaveAddr, unsigned char memAddr, int byteCount, unsigned char* data, int* cnt);


#endif /* SOURCES_I2C_H_ */
