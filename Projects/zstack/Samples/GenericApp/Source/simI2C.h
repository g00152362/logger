#ifndef __I2C_SIM_H__
#define __I2C_SIM_H__



#define SCL P1_0                //����P1.0��ΪSCL
#define SDA P1_1                //����P1.A��ΪSDA

#define SDA_HIGH  SDA=1
#define SCL_HIGH  SCL=1

#define SDA_LOW  SDA=0
#define SCL_LOW  SCL=0

typedef unsigned int   uint;
//typedef unsigned char  uint8;


 void udelay(uint Time);
 void set_scl_output(void);
 void set_sda_output(void);
 void set_scl_input(void);
 void set_sda_input(void);
 
 int simI2c_start(void);
 void simI2c_stop(void); 
 uint8 simI2c_read_ack(void);  
 void simI2c_send_ack(void);  
 void simI2c_write_byte(unsigned char b); 
 uint8 simI2c_read_byte(void); 
 void simI2c_read(unsigned char addr, unsigned char* buf, int len);  
 extern void simI2c_write (unsigned char addr, unsigned char* buf, int len);


#endif