/* Library includes. */
#include <ioCC2530.h>
#include "OnBoard.h"
#include "simI2C.h"

#define DELAY_TIME_IC 0x0010
#define DELAY2 0x001


void udelay(uint Time)
{
  int i;
  while(Time--)
  {
    for(i=0;i<100;i++)
        MicroWait(1);
  }
}




#if 0
void set_gpio_direction(uint16_t pin, unsigned char mode)
{
	GPIO_InitTypeDef GPIO_InitStructure;	

	GPIO_InitStructure.GPIO_Pin = pin;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = mode;
	GPIO_Init(IPORT, &GPIO_InitStructure);	
}


unsigned char get_gpio_value(uint16_t pin){

	return GPIO_ReadInputDataBit(IPORT,pin);
}


void simI2c_master_init(void)
{
	GPIO_DeInit(IPORT);
	RCC_APB2PeriphClockCmd( RCC_PORT , ENABLE); 	
	set_gpio_direction(SDA | SCL,OUTP);

}

#endif

// if the pin define is changed,the following function must be changed
void set_scl_output(void)
{
   P1SEL &= 0xFE;  //p1.0 
   P1DIR |= 0x01;
}

void set_sda_output(void)
{
   P1SEL &= 0xFD;  //p1.1
   P1DIR |= 0x02;
}

void set_scl_input(void)
{

    P1SEL &= ~0x01;     //����P1.0Ϊ��ͨIO��  
    P1DIR &= ~0x01;     //��P1.0Ϊ����ģʽ 
    P1INP &= ~0x01;     //��P1.0�������� ???
}

void set_sda_input(void)
{

    P1SEL &= ~0x02;     //����P1.1Ϊ��ͨIO��  
    P1DIR &= ~0x02;     //��P1.1Ϊ����ģʽ 
    P1INP &= ~0x02;     //��P1.1�������� ???
}


/* I2C��ʼ���� */  
int simI2c_start(void)  
{

    set_scl_output();
    set_sda_output();
	

    SDA_HIGH;             //����SDAΪ�ߵ�ƽ  
    SCL_HIGH;             //����SCLΪ�ߵ�ƽ  
    udelay(DELAY_TIME_IC);          //��ʱ  
        		 //��ʼ����  
    SDA_LOW;            //SCLΪ�ߵ�ƽʱ��SDA�ɸ߱��  
    udelay(DELAY_TIME_IC); 

    return 1;
} 





/* I2C��ֹ���� */  
void simI2c_stop(void)  
{  
    set_sda_output(); 

  //SCL�ߵ�ƽʱ��SDA�ɵͱ��  
    SCL_HIGH;
    SDA_LOW;
    udelay(DELAY_TIME_IC); 
    SDA_HIGH;     
}



  
/*   
I2C��ȡACK�ź�(д����ʱʹ��)  
����ֵ ��0��ʾACK�ź���Ч����0��ʾACK�ź���Ч  
*/  
uint8 simI2c_read_ack(void)  
{  
    uint8 r =0;  
    set_sda_input();        //����SDA����Ϊ����  

    SCL_HIGH;    // SCL���  
    udelay(DELAY_TIME_IC);  
    r = SDA;            //��ȡACK�ź�  
    SCL_LOW;		

    return r;  
} 


/* I2C����ACK�ź�(������ʱʹ��) */  
void simI2c_send_ack(void)  
{  
    set_sda_output(); 

    SCL_LOW;                                // SCL���  
    SDA_LOW;                                //����ACK�ź�  
    udelay(DELAY_TIME_IC);  
    SCL_HIGH;             // SCL���  
    udelay(DELAY_TIME_IC);  
} 


/* I2C�ֽ�д */  
void simI2c_write_byte(unsigned char b)  
{  
    int i; 
    uint8 r = 0;  
    set_sda_output(); 
  
    for (i=7; i>=0; i--) 
    {  
            SCL_LOW;             // SCL���  
            udelay(DELAY_TIME_IC);  
  
            //�Ӹ�λ����λ����׼�����ݽ��з���  
            if(b & (1<<i))
                    SDA_HIGH;
            else
                    SDA_LOW;
            
            udelay(DELAY2);  // guojun
  
            SCL_HIGH;
  
            udelay(DELAY_TIME_IC);  
    }  
    SCL_LOW;
    SDA_HIGH; 
            
    udelay(DELAY_TIME_IC); 	
    
    r = simI2c_read_ack();				   //���Ŀ���豸��ACK�ź�  
    //0:ACK 1:NCK 
    if(r)
    {
          HalUARTWrite(0,"nak?!?",4);   
    }
} 


/* I2C�ֽڶ� */  
uint8 simI2c_read_byte(void)  
{  
    int i;  
    uint8 r = 0; 
    
    set_sda_input(); 
    for (i=7; i>=0; i--) 
    {
        SCL_LOW;         // SCL���  
        udelay(DELAY_TIME_IC);  
        r = (r <<1) | SDA;      //�Ӹ�λ����λ����׼�����ݽ��ж�ȡ  
        SCL_HIGH;         // SCL���  
        udelay(DELAY_TIME_IC);  
    }  
    simI2c_send_ack();                 //��Ŀ���豸����ACK�ź�  
    return r;  
} 


/*  
I2C������  
addr��Ŀ���豸��ַ  
buf����������  
len�������ֽڵĳ���  
*/  
void simI2c_read(unsigned char addr, unsigned char* buf, int len)  
{  
	int i;  
	uint8 t;  
	simI2c_start();                        //��ʼ��������ʼ����ͨ��  
	
	//���͵�ַ�����ݶ�д����  
	t = (addr << 1) | 1;                    //��λΪ1����ʾ������  
	simI2c_write_byte(t);  

	//��������  
	for (i=0; i<len; i++)  
	buf[i] = simI2c_read_byte();  
	simI2c_stop();                     //��ֹ��������������ͨ��  
}

  
/*  
I2Cд����  
addr��Ŀ���豸��ַ  
buf��д������  
len��д���ֽڵĳ���  
*/  
void simI2c_write (unsigned char addr, unsigned char* buf, int len)  
{  
	int i;  
	unsigned char t;  
	simI2c_start();                        //��ʼ��������ʼ����ͨ��  

	//���͵�ַ�����ݶ�д����  
	t = (addr << 1) | 0;                    //��λΪ0����ʾд����  
	simI2c_write_byte(t);  

	//д������  
	for (i=0; i<len; i++)  
		simI2c_write_byte(buf[i]);  
	simI2c_stop();                     //��ֹ��������������ͨ��  
} 

