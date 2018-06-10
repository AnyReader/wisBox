/* DHT11 temperature sensor library
   Usage:
   		Set DHT PIN using  setDHTPin(pin) command
   		getFtemp(); this returns temperature in F
   Sam Johnston 
   October 2016
   This example code is in the Public Domain (or CC0 licensed, at your option.)
   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "rom/ets_sys.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
#include "dht11.h"

int DHT_PIN = 14;

int humidity = 0;
int temperature = 0;
int Ftemperature = 0;

int DHT_DATA[3] = {0,0,0};


void setDHTPin(int PIN)
{
	DHT_PIN = PIN;
	gpio_pad_select_gpio(DHT_PIN);
	gpio_set_pull_mode(DHT_PIN,GPIO_PULLUP_ONLY);
}
void errorHandle(int response)
{
	switch(response) {
	
		case DHT_TIMEOUT_ERROR :
			printf("DHT Sensor Timeout!\n");
		case DHT_CHECKSUM_ERROR:
			printf("CheckSum error!\n");
		case DHT_OKAY:
			break;
		default :
			printf("Dont know how you got here!\n");
	}
	temperature = 0;
	humidity = 0;
			
}

//new

#define		DHT11_PortInMode()	gpio_set_direction(DHT_PIN, GPIO_MODE_INPUT)  //Set the DHT11 Pin Input Mode
#define		DHT11_Pin_In()		gpio_get_level(DHT_PIN)

#define		DHT11_PortOutMode()		gpio_set_direction(DHT_PIN, GPIO_MODE_OUTPUT) //Set the DHT11 Pin Output Mode
#define		DHT11_Pin_Low()			gpio_set_level(DHT_PIN,0)
#define		DHT11_Pin_High()		gpio_set_level(DHT_PIN,1)
#define		DHT11_Delay_us(t)		ets_delay_us(t)

#define  True   0x01
#define  False  0x00

/******************************************************
 * FunctionName : StartDHT11
 * Description  : Start to read DHT11
 * Parameters   : none
 * Returns      : Start success return true
*******************************************************/
uint8_t StartDHT11(void)
{
   uint8_t result = 0;

   DHT11_PortOutMode();
   DHT11_Pin_Low();
   DHT11_Delay_us(25000);
   DHT11_Pin_High();
   DHT11_Delay_us(30);//20~40
   DHT11_PortInMode();
   DHT11_Delay_us(5);
   while(DHT11_Pin_In()==0)//DTH 80us 响应
   {
	 result++;
	 DHT11_Delay_us(10);
	 if(result>200) //timeout
		 return False;
   }
   result = 0;
   while(DHT11_Pin_In()==1)
   {
	   result++;//DTH 80us 响应
	   DHT11_Delay_us(10);
	   if(result>200) //
	  		 return False;
   }
   return True;

}




/******************************************************
 * FunctionName : DHT11_ReadByte
 * Description  : Read DHT11 one Byte
 * Parameters   : none
 * Returns      : ReadByte
*******************************************************/
int8_t DHT11_ReadByte(void)
{
	uint8_t reader=0;
	uint8_t bitsum;

	uint8_t  count = 0;
	DHT11_PortInMode();
	for(bitsum=0;bitsum<8;bitsum++)
	{
	  reader = reader << 1;
	  while(DHT11_Pin_In()==0)
	  {
		  count++;
		  DHT11_Delay_us(10);
		  if(count>200)
			 return -1;
	  }
	  DHT11_Delay_us(50);//
	  if(DHT11_Pin_In())
	     reader |= 0x01;
	  while(DHT11_Pin_In())//等待变低开始下一bit
	  {
		  count++;
		  DHT11_Delay_us(10);
		  if(count>200)
			 return -1;
	  }
	}
	return reader;
}

/******************************************************
 * FunctionName : ReadDHT11
 * Description  : Read DHT11 one Byte
 * Parameters   : result array(len == 4)
 * Returns      : Read DHT11 data Success then return 1
*******************************************************/
uint8_t ReadDHT11(uint8_t *dht)
{
	uint8_t sum;
    uint8_t checksum=0;
	if(StartDHT11()==True)
	{
		dht[0]   = DHT11_ReadByte();
		dht[1]   = DHT11_ReadByte();
		dht[2]   = DHT11_ReadByte();
		dht[3]   = DHT11_ReadByte();
		checksum = DHT11_ReadByte();
		sum = (dht[0]+dht[1]+dht[2]+dht[3]);
		if(checksum==sum)
			return True;
		else
			return False;
	};
	return False;
}

/******************************************************
 * FunctionName : DHT11_NumToString
 * Description  : change a byte to Dec String
 * Parameters   : dht: the num; tr: Result String Point
 * Returns      : none
*******************************************************/
void DHT11_NumToString(uint8_t dht, uint8_t *str)
{
  str[0] = (dht%100)/10 + '0';//十位
  str[1] = dht%10 + '0';//个位
  str[2] = '\0';
}
