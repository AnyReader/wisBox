#ifndef BH1750_H_  
#define BH1750_H_

#include "stdint.h"

#define BH1750_Addr 	0x23//0x46
#define BH1750_ON   0x01
#define BH1750_CON  0x10
#define BH1750_ONE  0x20
#define BH1750_RSET 0x07

uint8_t Init_BH1750(int pin_sda, int pin_scl);
//uint8_t Cmd_Write_BH1750(uint8_t cmd);
//void Start_BH1750(void);

uint16_t Read_BH1750(void);
float Convert_BH1750(void);


#endif
