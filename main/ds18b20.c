/*
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
     along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "driver/gpio.h"
#include "rom/ets_sys.h"


#include "ds18b20.h"


#define		ds18b20_PortInMode()	gpio_set_direction(DS_GPIO, GPIO_MODE_INPUT)  //Set the DHT11 Pin Input Mode
#define		ds18b20_Pin_In()		gpio_get_level(DS_GPIO)

#define		ds18b20_PortOutMode()		gpio_set_direction(DS_GPIO, GPIO_MODE_OUTPUT) //Set the DHT11 Pin Output Mode
#define		ds18b20_Pin_Low()			gpio_set_level(DS_GPIO,0)
#define		ds18b20_Pin_High()		gpio_set_level(DS_GPIO,1)
#define		ds18b20_Delay_us(t)		ets_delay_us(t)

#define  True   0x01
#define  False  0x00


int init=0;
/// Sends one bit to bus
void ds18b20_send(char bit){
  gpio_set_direction(DS_GPIO, GPIO_MODE_OUTPUT);
  gpio_set_level(DS_GPIO,0);
  ets_delay_us(5);
  if(bit==1)gpio_set_level(DS_GPIO,1);
  ets_delay_us(80);
  gpio_set_level(DS_GPIO,1);
}
// Reads one bit from bus
unsigned char ds18b20_read(void){
  unsigned char PRESENCE=0;
  gpio_set_direction(DS_GPIO, GPIO_MODE_OUTPUT);
  gpio_set_level(DS_GPIO,0);
  ets_delay_us(2);
  gpio_set_level(DS_GPIO,1);
  ets_delay_us(15);
  gpio_set_direction(DS_GPIO, GPIO_MODE_INPUT);
  if(gpio_get_level(DS_GPIO)==1) PRESENCE=1; else PRESENCE=0;
  return(PRESENCE);
}
// Sends one byte to bus
void ds18b20_send_byte(char data){
  unsigned char i;
  unsigned char x;
  for(i=0;i<8;i++){
    x = data>>i;
    x &= 0x01;
    ds18b20_send(x);
  }
  ets_delay_us(100);
}
// Reads one byte from bus
unsigned char ds18b20_read_byte(void){
  unsigned char i;
  unsigned char data = 0;
  for (i=0;i<8;i++)
  {
    if(ds18b20_read()) data|=0x01<<i;
    ets_delay_us(15);
  }
  return(data);
}
// Sends reset pulse
unsigned char ds18b20_RST_PULSE(void){
  unsigned char PRESENCE;
  gpio_set_direction(DS_GPIO, GPIO_MODE_OUTPUT);
  gpio_set_level(DS_GPIO,0);
  ets_delay_us(500);
  gpio_set_level(DS_GPIO,1);
  gpio_set_direction(DS_GPIO, GPIO_MODE_INPUT);
  ets_delay_us(30);
  if(gpio_get_level(DS_GPIO)==0) PRESENCE=1; else PRESENCE=0;
  ets_delay_us(470);
  if(gpio_get_level(DS_GPIO)==1) PRESENCE=1; else PRESENCE=0;
  return PRESENCE;
}
// Returns temperature from sensor
float ds18b20_get_temp(void) {
  if(init==1){
    unsigned char check;
    char temp1=0, temp2=0;
      check=ds18b20_RST_PULSE();
      if(check==1)
      {
        ds18b20_send_byte(0xCC);
        ds18b20_send_byte(0x44);
        vTaskDelay(750 / portTICK_RATE_MS);
        check=ds18b20_RST_PULSE();
        ds18b20_send_byte(0xCC);
        ds18b20_send_byte(0xBE);
        temp1=ds18b20_read_byte();
        temp2=ds18b20_read_byte();
        check=ds18b20_RST_PULSE();
        float temp=0;
        temp=(float)(temp1+(temp2<<8))/16;
        return temp;
      }
      else{return 0;}

  }
  else{return 0;}
}
void ds18b20_init(int GPIO){
//  DS_GPIO = GPIO;
  gpio_pad_select_gpio(DS_GPIO);
  gpio_set_pull_mode(DS_GPIO,GPIO_PULLUP_ONLY);
  init=1;
}



//new
unsigned char Ds18b20ReadByte(){
	uint8_t j;
	unsigned char bi=0,byte=0x00;
	for(j=0;j<8;j++){
		ds18b20_PortOutMode();
		ds18b20_Pin_Low();//gpio_output_set(0,BIT5,BIT5,0);//先将总线拉低
		ds18b20_Delay_us(4);
		ds18b20_Pin_High();
		ds18b20_Delay_us(1);
		ds18b20_PortInMode();//gpio_output_set(0,0,0,BIT5);//释放总线
		ds18b20_Delay_us(9);
		bi=ds18b20_Pin_In();//GPIO_INPUT_GET(5);
		byte=(byte>>1) | (bi<<7);//bytet|=(bi<<j);//bytet=( (bi<<j));//bytet|=bi<<i;
		ds18b20_Delay_us(51);

	}
	return byte;
}

void Ds18b20WriteByte(uint8_t dat){
	uint8_t j;
	ds18b20_PortOutMode();
	for(j=0;j<8;j++)
	{
		ds18b20_Pin_Low();//gpio_output_set(0,BIT5,BIT5,0);//先将总线拉低
		ds18b20_Delay_us(5);
		if(dat & 0x01){
			ds18b20_Pin_High();//gpio_output_set(BIT5,0,BIT5,0);//写入1
		}else{
			ds18b20_Pin_Low();//gpio_output_set(0,BIT5,BIT5,0);//写入0；
		}
		ds18b20_Delay_us(68);//??? 60us
		ds18b20_Pin_High();
		ds18b20_PortInMode();//gpio_output_set(0,0,0,BIT5);//释放总线

		dat>>=1;
		ds18b20_Delay_us(5);

	}

}

bool Ds18b20_init(){
	unsigned int i=0;
	ds18b20_PortOutMode();
	ds18b20_Pin_Low();//先将总线拉低
	ds18b20_Delay_us(642);
	ds18b20_Pin_High();
	ds18b20_PortInMode();//gpio_output_set(BIT5,0,BIT5,BIT5);//释放总线
	ds18b20_Delay_us(1);
	while(ds18b20_Pin_In()){
		ds18b20_Delay_us(60);
		i++;//60-240us is wright
		if(i>5)
		{
			return false;
		}
	}
	//turn high
	return true;

}


int ds18b20ReadTemp(){

	unsigned char temp1;
	unsigned char temp2;

	float tp;
	int temp=0;
	unsigned char tml,tmh;

//	Ds18b20ChangeTemp();//先写入转换命令

	Ds18b20_init();//对ds18b20初始化
	ds18b20_Delay_us(250);
	Ds18b20WriteByte(0xcc);//跳过ROM操作命令
	Ds18b20WriteByte(0x44);//温度转换命令
	ds18b20_Delay_us(1000);

	ds18b20_Delay_us(250);
//	Ds18b20ReadTempCom();//读取温度

	Ds18b20_init();//对ds18b20初始化
	ds18b20_Delay_us(250);
	Ds18b20WriteByte(0xcc);//跳过ROM操作命令
	Ds18b20WriteByte(0xbe);//温度转换命令

	tml=Ds18b20ReadByte();//读取低位
	tmh=Ds18b20ReadByte();//读取高位

	temp=(tmh<<8)+tml;

	//正值情况
	if(temp>=0)//(tmh & 0xf0 == 0x00)
	{
		temp2=tml & 0x0f;//取低四位（精度）
		temp1=(tml>>4)|(tmh<<4);
		printf("A-当前温度：%d.%-4d℃\r\n",temp1,(temp2*625)); // 小数点后取4位精度  *0.0625*10000   格式输出4位25.0625 小心成25.625
	}

	else //if (tmh & 0xf0 == 0xf0)//负值
	{
		temp=(tmh<<8)+tml;
		temp=~temp;
		temp++;
		temp2=temp&0x0f;
		temp1=(temp>>4); //temp&0x0fff多余 取反后高4位已经为0
		printf("\nA-当前温度：-%d.%-4d℃\r\n",temp1,(temp2*625));
	}


	temp=tmh;
	temp<<=8;
	temp |=tml;

	if(temp<0)
	{
		temp--;
		temp=~temp;
		tp=temp;
		temp=tp*100*0.0625+0.5;//放大100倍、小数点后1位四舍五入
	}else
	{
		tp=temp;
		temp=tp*100*0.0625+0.5;

	}
	return temp;
}
