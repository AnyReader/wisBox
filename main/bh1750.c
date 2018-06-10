

#include "bh1750.h"
#include <stdbool.h>
#include <stdio.h>
#include "../../components/camera/twi.h"
#include "rom/ets_sys.h"



#define 	_delay_ms(ms)  ets_delay_us(1000*ms)

/*************************************************************
  Function   :Cmd_Write_BH1750
  Description:дBH1750 ����
  Input      : cmd ---����
  return     : none
*************************************************************/
uint8_t Cmd_Write_BH1750(uint8_t cmd)
{
//	I2C_Start();                  //��ʼ�ź�
//    I2C_WriteByte(BH1750_Addr+0); //�����豸��ַ+д�ź�

//	I2C_WaiteForAck();
//    I2C_WriteByte(cmd);           //�ڲ��Ĵ�����ַ
//	I2C_WaiteForAck();
//	I2C_Stop();                   //����ֹͣ�ź�
//

    uint8_t ret=0;

//    __disable_irq();
    if(twi_writeTo(BH1750_Addr, &cmd, 1, true) != 0) {
        ret=0xFF;
    }
//    __enable_irq();
    if (ret != 0) {
        printf("BH1750_Write [%02x] failed\n", cmd);
    }
    _delay_ms(5);
    return ret;

}
/*************************************************************
  Function   : Start_BH1750
  Description: ����BH1750
  Input      : none
  return     : none
*************************************************************/
void Start_BH1750(void)
{
	Cmd_Write_BH1750(BH1750_CON);  //һ��H�ֱ���ģʽ������120ms
	_delay_ms(180);
}


/*************************************************************
  Function   :Init_BH1750
  Description:��ʼ��BH1750
  Input      : none
  return     : none
*************************************************************/
void Init_BH1750(int pin_sda, int pin_scl)
{

//	twi_init(pin_sda, pin_scl);//I2Cx_Init();
	Cmd_Write_BH1750(BH1750_ON);	   //power on
	Cmd_Write_BH1750(BH1750_RSET);	 //clear

}

/*************************************************************
  Function   : Read_BH1750
  Description: BH1750������ֵ
  Input      :  none
  return     : ---���ص�����  uint16_t
*************************************************************/
uint16_t Read_BH1750(void)
{
	uint8_t data[2];
	uint16_t temp;

 //   I2C_Start();                         //��ʼ�ź�
//    I2C_WriteByte(BH1750_Addr+1);        //�����豸��ַ+���ź�
//	I2C_WaiteForAck();
//	temp[0]=I2C_ReadByte(I2C_ACK);       //ACK read data msb first
//	temp[1]=I2C_ReadByte(I2C_NACK);      //NACK
//	I2C_Stop();                          //ֹͣ�ź�
//	data = temp[0];
//	data = (data<<8)+temp[1];			//�ϳ����ݣ�����������
//    delay_ms(5);

	Cmd_Write_BH1750(BH1750_ON);	   //power on
	_delay_ms(20);
	Cmd_Write_BH1750(BH1750_CON);  //һ��H�ֱ���ģʽ������120ms
	_delay_ms(180);


//    __disable_irq();
    int rc = twi_readFrom(BH1750_Addr, &data, 2, true);
    if (rc != 0)
    {
            return 0xFF;
     }

//    __enable_irq();
    if (rc != 0) {
        printf("BH1750_Read [%02x] failed rc=%d\n", data[0], rc);
    }
    temp=(data[0]<<8)+data[1];

    return (temp);


}
/*************************************************************
  Function   : Convert_BH1750
	Description: ת����ȡ�Ĺ���ֵ��unit:lx ����0.1 lx
  Input      : light_value ---  bh1750��register���ն�ֵ
	return     : ת����Ĺ��ն� unit :lx
*************************************************************/
float Convert_BH1750(void)
{

	uint16_t light_value=Read_BH1750();
	float result_lx=0;

	result_lx=(float)light_value/1.2;

	return result_lx;
}
