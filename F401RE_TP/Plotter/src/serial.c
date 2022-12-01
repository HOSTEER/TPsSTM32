/*
 * serial.c
 *
 *  Created on: Jun 15, 2022
 *      Author: laurmonc
 */
#include <string.h>
#include <stdio.h>
#include "stm32f4xx_hal.h"

extern UART_HandleTypeDef huart2;
char gbuf[20];


void SERIAL_SendCharBuf(char *buf){
	int taille=0;
	taille=strlen(buf);
	if(taille>0)
		HAL_UART_Transmit(&huart2, (unsigned char *)buf, taille, 1);
}

void SERIAL_SendInt(int n){
	int taille=0;
	taille=sprintf (gbuf,"%d",n);
	if(taille>0)
		HAL_UART_Transmit(&huart2, (unsigned char *)gbuf, taille, 1);
}

void SERIAL_SendFloat(float v){
	int taille=0;

	taille=sprintf (gbuf, "%3.3f",v); // @suppress("Float formatting support")
	if(taille>0)
		HAL_UART_Transmit(&huart2, (unsigned char *)gbuf, taille, 1);
}

void SERIAL_SendNL(){
	gbuf[0]=13;
	gbuf[1]=10;
	HAL_UART_Transmit(&huart2, (unsigned char *)gbuf, 2, 1);

}

void SERIAL_SendTAB(){
	gbuf[0]=9;

	HAL_UART_Transmit(&huart2, (unsigned char *)gbuf, 1, 1);

}

void SERIAL_SendToPlot(int * dataA,int *dataB,int nb){
	int n=0;
	for(n=0;n<nb;n++){
		SERIAL_SendCharBuf("a:");
		SERIAL_SendInt(dataA[n]);
		SERIAL_SendCharBuf(",");
		SERIAL_SendCharBuf("b:");
		SERIAL_SendInt(dataB[n]);
		SERIAL_SendNL();
	}
}

void SERIAL_SendFloatToPlot(double dataA, double dataB){
	SERIAL_SendCharBuf("a:");
	SERIAL_SendFloat(dataA);
	SERIAL_SendCharBuf(",");
	SERIAL_SendCharBuf("b:");
	SERIAL_SendFloat(dataB);
	SERIAL_SendNL();
}
