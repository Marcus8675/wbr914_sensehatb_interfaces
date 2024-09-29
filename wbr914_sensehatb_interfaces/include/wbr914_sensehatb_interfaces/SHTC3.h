#ifdef __cplusplus
extern "C" {
#endif

#ifndef _SHTC3_H
#define _SHTC3_H

#include <lgpio.h>

//i2c address
#define SHTC3_I2C_ADDRESS	    	0x70

//Commands
#define SHTC3_WakeUp                0x3517
#define SHTC3_Sleep                 0xB098
#define SHTC3_NM_CE_ReadTH          0x7CA2
#define SHTC3_NM_CE_ReadRH          0x5C24
#define SHTC3_NM_CD_ReadTH          0x7866
#define SHTC3_NM_CD_ReadRH          0x58E0
#define SHTC3_LM_CE_ReadTH          0x6458
#define SHTC3_LM_CE_ReadRH          0x44DE
#define SHTC3_LM_CD_ReadTH          0x609C
#define SHTC3_LM_CD_ReadRH          0x401A
#define SHTC3_Software_RES          0x805D
#define SHTC3_ID               0xEFC8

#define CRC_POLYNOMIAL              0x131 // P(x) = x^8 + x^5 + x^4 + 1 = 100110001

#endif

void SHTC3_Read_DATA(float *TH, float *RH);
char SHTC3_CheckCrc(char data[], unsigned char len, unsigned char checksum);
int SHTC3_WriteCommand(unsigned short cmd);
void SHTC3_WAKEUP();
void SHTC3_SLEEP();
void SHTC_SOFT_RESET();

#ifdef __cplusplus
}
#endif