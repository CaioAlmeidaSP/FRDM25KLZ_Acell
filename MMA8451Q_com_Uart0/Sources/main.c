/*
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of Freescale Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "MKL25Z4.h"
#include "I2C.h"

void delay_us(unsigned int tempo);
void delay_ms(unsigned int tempo);

void Uart0_init(void);
void enviar_frame_uart(char* frame,char ultimo);
void enviar_byte_uart(unsigned char byte);

unsigned short int complemento2(unsigned short int);

void MMA845x_Standby (void);
void MMA845x_Active (void);
void MMA845x_init (void);

void enviar_eixo_uart(unsigned short int valor,unsigned char eixo);
void enviar_eixo_signal_uart(unsigned short int valor,unsigned char eixo);

#define MMA8451Q_ADDR 0x1D
#define CTRL_REG1	0x2A
#define CTRL_REG2	0x2B
#define XYZ_DATA_CFG_REG	0x0E



static int i = 0;

int main(void)
{

    /* Write your code here */
	unsigned char acelerometro[7];
	unsigned short int eixoX,eixoY,eixoZ;
	int count;
	int erro;

	I2C0_int();
	Uart0_init();
	MMA845x_init ();


	//Inicialização pinos do RGB
	SIM_SCGC5 |= 1<<10 | 1<<12; //Habilitando o Clock para o PortB

	PORTB_PCR18 |= 1<<8 | 1<<6 | 1<<2;
	PORTB_PCR18 &= ~(1<<4) & ~(1<<1) & ~(1<<0);

	PORTB_PCR19 |= 1<<8 | 1<<6 | 1<<2;
	PORTB_PCR19 &= ~(1<<4) & ~(1<<1) & ~(1<<0);

	PORTD_PCR1 |= 1<<8 | 1<<6 | 1<<2;
	PORTD_PCR1 &= ~(1<<4) & ~(1<<1) & ~(1<<0);

	GPIOB_PDDR |= 1<<18;
	GPIOB_PSOR |= 1<<18;

	GPIOB_PDDR |= 1<<19;
	GPIOB_PSOR |= 1<<19;

	GPIOD_PDDR |= 1<<1;
	GPIOD_PSOR |= 1<<1;


	enviar_frame_uart("Programa Acelerometro MMA8451Q \n",0x00);


    /* This for loop should be replaced. By default this loop allows a single stepping. */
    for (;;) {
    	erro = I2C0_MultiByteRead(MMA8451Q_ADDR, 0x00, 7, acelerometro, &count);//Faz a leitura dos registros

    	eixoX = acelerometro[1]<<8 | acelerometro[2];
    	//enviar_eixo_signal_uart(eixoX,'X');//Envia para serial com sinal
    	////enviar_eixo_uart((eixoX>>2),'X');//Enviar  valor lido integral
    	eixoX = complemento2(eixoX);

    	enviar_eixo_uart(eixoX,'X');
    	//eixoX = acelerometro[1];

   		eixoY = acelerometro[3]<<8 | acelerometro[4];
   		//enviar_eixo_signal_uart(eixoY,'Y');//Envia para serial com sinal
   		////enviar_eixo_uart((eixoY>>2),'Y');//Enviar  valor lido integral
   		eixoY = complemento2(eixoY);
   		//eixoY = acelerometro[3];

   		enviar_eixo_uart(eixoY,'Y');

   		eixoZ = acelerometro[5]<<8 | acelerometro[6];
   		//enviar_eixo_signal_uart(eixoZ,'Z');//Envia para serial com sinal
   		////enviar_eixo_uart((eixoZ>>2),'Z');//Enviar  valor lido integral
   		eixoZ = complemento2(eixoZ);
   		//eixoZ = acelerometro[5];

   		enviar_eixo_uart(eixoZ,'Z');


//Teste do git
   		//Acionamento dos LEDs conforme os eixos
   		if(eixoX>0x07FF)
   			GPIOB_PCOR |= 1<<18; //Liga Led Vermelho
   		else
   			GPIOB_PSOR	|= 1<<18;

   		if(eixoY>0x07FF)
   			GPIOB_PCOR |= 1<<19; //Liga led verde
   		else
   			GPIOB_PSOR	|= 1<<19;

   		if(eixoZ<0x05FF)
   			GPIOD_PCOR |= 1<<1; //Liga led azul
   		else
   			GPIOD_PSOR |= 1<<1;



   		delay_ms(160);

   		//enviar_byte_uart('\n');
   		//enviar_byte_uart('\b');

   		i++;
    }
    /* Never leave main */
    return 0;
}

void enviar_eixo_uart(unsigned short int valor,unsigned char eixo){
	unsigned char ucMask[] = {'0','1','2','3','4','5','6','7','8','9','A','B','C','D','E','F'};
	unsigned char frame_eixo[8];

	frame_eixo[0]=0x02;
	frame_eixo[1]=eixo;

	frame_eixo[5]= ucMask[valor%16];
	valor/=16;
	frame_eixo[4]= ucMask[valor%16];
	valor/=16;
	frame_eixo[3]= ucMask[valor%16];
	valor/=16;
	frame_eixo[2]= ucMask[valor%16];
	valor/=16;

	frame_eixo[6]=0x04;
	frame_eixo[7]=0x00;

	enviar_frame_uart(frame_eixo,0x00);
}

void enviar_eixo_signal_uart(unsigned short int valor,unsigned char eixo){
	unsigned char ucMask[] = {'0','1','2','3','4','5','6','7','8','9','A','B','C','D','E','F'};
	unsigned char frame_eixo[9];
	unsigned char sinal;
	unsigned short int valor_enviar;

	if(valor > 0x7FFC){
	sinal='-';
	valor_enviar = (((~valor) + 1)>>2)& 0x3FFF;
	}
	else{
		sinal='+';
	valor_enviar = (valor>>2)& 0x3FFF;
	}

	frame_eixo[0]=0x02;
	frame_eixo[1]=eixo;
	frame_eixo[2]=sinal;

	frame_eixo[6]= ucMask[valor_enviar%16];
	valor_enviar/=16;
	frame_eixo[5]= ucMask[valor_enviar%16];
	valor_enviar/=16;
	frame_eixo[4]= ucMask[valor_enviar%16];
	valor_enviar/=16;
	frame_eixo[3]= ucMask[valor_enviar%16];
	valor_enviar/=16;

	frame_eixo[7]=0x04;
	frame_eixo[8]=0x00;

	enviar_frame_uart(frame_eixo,0x00);
}

void Uart0_init(void){


	//INICIALIZANDO O PERIFÉRICO UART 0
	SIM->SCGC4 |= 1<<10; // HABILITA CLOCK PARA O PERIFERICO, PAG:204(REFERNCE MANUAL)
	SIM->SOPT2 |= 1<<26;//CONFIGURANDO O CLOCK PARA VALOR DO CLOCK DO PROCESSADOR, PAG:196
	SIM->SOPT2 &= ~(1<<27);
	UART0->C2 =0x00; //DESABILITA O FUNCIONAMENTO DA UART0, PAG728
	UART0->BDH =0X01;// calculo para definição do BaudRate - Neste caso, 48.000.000Hz/16/9600=312=0x138
	UART0->BDL=0x38;
	UART0->C4=0X0F;//SELECIONA MODO 8BITS, E CONFIGURA OVER SAMPLING PARA 16 PAG:736 E 737
	UART0->C2 |= 1<<3; //HABILITA TRANSMIÇÃO DA UART
	UART0->C2 |= 1<<2; //HABILITA TRANSMIÇÃO DA UART


	//INICIALIZANDO OS PINOS DA UART0 NO PA1 E PA2 (OPENSDA)

	SIM->SCGC5 |= 1<<9; //HABILITA CLOCK NO PORTA , PAG:206

	PORTA_PCR1 = 0x00000200;//Inicializa PortA.1 como Alternative Function 2 (Uart0_Tx)
	PORTA_PCR2 = 0x00000200;//Inicializa PortA.2 como Alternative Function 2 (Uart0_Tx)
}

void enviar_byte_uart(unsigned char byte){
	UART0->D = byte; //Envio do byte
	while(!(UART0->S1 & 0x80));//Aguarda termino o envio
}

void enviar_frame_uart(char* frame,char ultimo){
	char t=0;

	while(frame[t]!=ultimo){
			UART0->D = frame[t]; //Envio de caracter
			while(!(UART0->S1 & 0x80));
			t++;
		}

	//do{
	//	UART0->D = frame[t]; //Envio de caracter
	//	while(!(UART0->S1 & 0x80));
	//	t++;
	//}
	//while(frame[t]!=ultimo);
}


void delay_ms(unsigned int tempo){
	int i;
	SysTick->LOAD = 47999;/*/delay = (N + 1) / sysclk
	(N + 1) = delay × sysclk = 0.001 sec × 48 MHz = 48000 ==> N = 48000 – 1 = 47999*/
	SysTick->CTRL = 0x5; /* Enable the timer and choose sysclk as the clock
	source */
	for(i = 0; i < tempo; i++) {
	while((SysTick->CTRL & 0x10000) == 0) /* wait until the COUTN flag is set */
	{ }
	}
	SysTick->CTRL = 0; /* Stop the timer (Enable = 0) */
	}

void delay_us(unsigned int tempo){
	int i;
	SysTick->LOAD = 48;/*/delay = (N + 1) / sysclk
	(N + 1) = delay × sysclk = 0.001 sec × 48 MHz = 48000 ==> N = 48000 – 1 = 47999*/
	SysTick->CTRL = 0x5; /* Enable the timer and choose sysclk as the clock
	source */
	for(i = 0; i < tempo; i++) {
	while((SysTick->CTRL & 0x10000) == 0) /* wait until the COUTN flag is set */
	{ }
	}
	SysTick->CTRL = 0; /* Stop the timer (Enable = 0) */
	}

unsigned short int complemento2(unsigned short int numero){
	if(numero > 0x7FFC){
		return (((~numero) + 1)>>2) & 0x3FFF;
	}
	else{
		return (numero>>2)& 0x3FFF;
	}
}


void MMA845x_Standby (void){
unsigned char n;
/*
** Read current value of System Control 1 Register.
** Put sensor into Standby Mode by clearing the Active bit
** Return with previous value of System Control 1 Register.
*/
//n = IIC_RegRead(CTRL_REG1);
I2C0_byteRead(MMA8451Q_ADDR, CTRL_REG1, &n);

//IIC_RegWrite(CTRL_REG1, n & ~ACTIVE_MASK);
I2C0_byteWrite(MMA8451Q_ADDR, CTRL_REG1, (n & 0xFE));

}

void MMA845x_Active (void){
	static char n;
	I2C0_byteRead(MMA8451Q_ADDR, CTRL_REG1, &n);
/*
** Set the Active bit in CTRL Reg 1
*/
	I2C0_byteWrite(MMA8451Q_ADDR, CTRL_REG1, (n | 0x01));
//IIC_RegWrite(CTRL_REG1, (IIC_RegRead(CTRL_REG1) | ACTIVE_MASK));
}

void MMA845x_init (void)
{
	static char n;

	MMA845x_Standby ();

	//Configura a resolução para 2g
	I2C0_byteRead(MMA8451Q_ADDR, XYZ_DATA_CFG_REG, &n);
	I2C0_byteWrite(MMA8451Q_ADDR, XYZ_DATA_CFG_REG, (n & 0xFC));

	//Configura amostragem para 6.25Hz, 160ms
	I2C0_byteRead(MMA8451Q_ADDR, CTRL_REG1, &n);
	I2C0_byteWrite(MMA8451Q_ADDR, CTRL_REG1, (n | 0x30));

	//Configura para alta definição
	I2C0_byteRead(MMA8451Q_ADDR, CTRL_REG2, &n);
	I2C0_byteWrite(MMA8451Q_ADDR, CTRL_REG2, (n | 0x02));

	/*
	**Put the part back into the Active Mode
	*/
	MMA845x_Active();

}
////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////
