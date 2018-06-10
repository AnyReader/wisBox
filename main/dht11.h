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
#ifndef DHT11_H_  
#define DHT11_H_

#define DHT_TIMEOUT_ERROR -2
#define DHT_CHECKSUM_ERROR -1
#define DHT_OKAY  0



// function prototypes

//Start by using this function
void setDHTPin(int PIN);

void errorHandle(int response);




uint8_t ReadDHT11(uint8_t *dht);

void DHT11_NumToString(uint8_t dht, uint8_t *str);
#endif
