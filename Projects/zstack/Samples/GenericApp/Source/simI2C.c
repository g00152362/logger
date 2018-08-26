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

    P1SEL &= ~0x01;     //设置P1.0为普通IO口  
    P1DIR &= ~0x01;     //设P1.0为输入模式 
    P1INP &= ~0x01;     //打开P1.0上拉电阻 ???
}

void set_sda_input(void)
{

    P1SEL &= ~0x02;     //设置P1.1为普通IO口  
    P1DIR &= ~0x02;     //设P1.1为输入模式 
    P1INP &= ~0x02;     //打开P1.1上拉电阻 ???
}


/* I2C起始条件 */  
int simI2c_start(void)  
{

    set_scl_output();
    set_sda_output();
	

    SDA_HIGH;             //设置SDA为高电平  
    SCL_HIGH;             //设置SCL为高电平  
    udelay(DELAY_TIME_IC);          //延时  
        		 //起始条件  
    SDA_LOW;            //SCL为高电平时，SDA由高变低  
    udelay(DELAY_TIME_IC); 

    return 1;
} 





/* I2C终止条件 */  
void simI2c_stop(void)  
{  
    set_sda_output(); 

  //SCL高电平时，SDA由低变高  
    SCL_HIGH;
    SDA_LOW;
    udelay(DELAY_TIME_IC); 
    SDA_HIGH;     
}



  
/*   
I2C读取ACK信号(写数据时使用)  
返回值 ：0表示ACK信号有效；非0表示ACK信号无效  
*/  
uint8 simI2c_read_ack(void)  
{  
    uint8 r =0;  
    set_sda_input();        //设置SDA方向为输入  

    SCL_HIGH;    // SCL变高  
    udelay(DELAY_TIME_IC);  
    r = SDA;            //读取ACK信号  
    SCL_LOW;		

    return r;  
} 


/* I2C发出ACK信号(读数据时使用) */  
void simI2c_send_ack(void)  
{  
    set_sda_output(); 

    SCL_LOW;                                // SCL变低  
    SDA_LOW;                                //发出ACK信号  
    udelay(DELAY_TIME_IC);  
    SCL_HIGH;             // SCL变高  
    udelay(DELAY_TIME_IC);  
} 


/* I2C字节写 */  
void simI2c_write_byte(unsigned char b)  
{  
    int i; 
    uint8 r = 0;  
    set_sda_output(); 
  
    for (i=7; i>=0; i--) 
    {  
            SCL_LOW;             // SCL变低  
            udelay(DELAY_TIME_IC);  
  
            //从高位到低位依次准备数据进行发送  
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
    
    r = simI2c_read_ack();				   //检查目标设备的ACK信号  
    //0:ACK 1:NCK 
    if(r)
    {
          HalUARTWrite(0,"nak?!?",4);   
    }
} 


/* I2C字节读 */  
uint8 simI2c_read_byte(void)  
{  
    int i;  
    uint8 r = 0; 
    
    set_sda_input(); 
    for (i=7; i>=0; i--) 
    {
        SCL_LOW;         // SCL变低  
        udelay(DELAY_TIME_IC);  
        r = (r <<1) | SDA;      //从高位到低位依次准备数据进行读取  
        SCL_HIGH;         // SCL变高  
        udelay(DELAY_TIME_IC);  
    }  
    simI2c_send_ack();                 //向目标设备发送ACK信号  
    return r;  
} 


/*  
I2C读操作  
addr：目标设备地址  
buf：读缓冲区  
len：读入字节的长度  
*/  
void simI2c_read(unsigned char addr, unsigned char* buf, int len)  
{  
	int i;  
	uint8 t;  
	simI2c_start();                        //起始条件，开始数据通信  
	
	//发送地址和数据读写方向  
	t = (addr << 1) | 1;                    //低位为1，表示读数据  
	simI2c_write_byte(t);  

	//读入数据  
	for (i=0; i<len; i++)  
	buf[i] = simI2c_read_byte();  
	simI2c_stop();                     //终止条件，结束数据通信  
}

  
/*  
I2C写操作  
addr：目标设备地址  
buf：写缓冲区  
len：写入字节的长度  
*/  
void simI2c_write (unsigned char addr, unsigned char* buf, int len)  
{  
	int i;  
	unsigned char t;  
	simI2c_start();                        //起始条件，开始数据通信  

	//发送地址和数据读写方向  
	t = (addr << 1) | 0;                    //低位为0，表示写数据  
	simI2c_write_byte(t);  

	//写入数据  
	for (i=0; i<len; i++)  
		simI2c_write_byte(buf[i]);  
	simI2c_stop();                     //终止条件，结束数据通信  
} 

