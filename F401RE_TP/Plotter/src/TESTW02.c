#include "serial.h"

void test(int aIntegerNumber, float aFloatNumber){
	SERIAL_SendInt(aIntegerNumber);
	SERIAL_SendTAB();
	SERIAL_SendFloat(aFloatNumber);
	SERIAL_SendNL();
}
