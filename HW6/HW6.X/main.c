#include<xc.h>           // processor SFR definitions
#include<sys/attribs.h>  // __ISR macro
#include <math.h>
#include "ILI9163C.h"
#include"i2c_master_noint.h"

#define SYS_FREQ 48000000 // system frequency 48MHz
#define PI 3.141592654

// DEVCFG0
#pragma config DEBUG = OFF // no debugging
#pragma config JTAGEN = OFF // no jtag
#pragma config ICESEL = ICS_PGx1 // use PGED1 and PGEC1
#pragma config PWP = OFF // no write protect
#pragma config BWP = OFF // no boot write protect
#pragma config CP = OFF // no code protect

// DEVCFG1
#pragma config FNOSC = PRIPLL // use primary oscillator with pll
#pragma config FSOSCEN = OFF // turn off secondary oscillator
#pragma config IESO = OFF // no switching clocks
#pragma config POSCMOD = HS // high speed crystal mode
#pragma config OSCIOFNC = OFF // free up secondary osc pins //??????????????????????????
#pragma config FPBDIV = DIV_1 // divide CPU freq by 1 for peripheral bus clock
#pragma config FCKSM = CSDCMD // do not enable clock switch
#pragma config WDTPS = PS1048576 // slowest wdt
#pragma config WINDIS = OFF // no wdt window
#pragma config FWDTEN = OFF // wdt off by default
#pragma config FWDTWINSZ = WINSZ_25 // wdt window at 25%

// DEVCFG2 - get the CPU clock to 48MHz
#pragma config FPLLIDIV = DIV_2 // divide input clock to be in range 4-5MHz
#pragma config FPLLMUL = MUL_24 // multiply clock after FPLLIDIV
#pragma config FPLLODIV = DIV_2 // divide clock after FPLLMUL to get 48MHz
#pragma config UPLLIDIV = DIV_2 // divider for the 8MHz input clock, then multiply by 12 to get 48MHz for USB
#pragma config UPLLEN = ON // USB clock on

// DEVCFG3
#pragma config USERID = 0x0001 // some 16bit userid, doesn't matter what
#pragma config PMDL1WAY = OFF // allow multiple reconfigurations
#pragma config IOL1WAY = OFF // allow multiple reconfigurations
#pragma config FUSBIDIO = ON // USB pins controlled by USB module
#pragma config FVBUSONIO = ON // USB BUSON controlled by USB module

void i2c_init(void){
    ANSELBbits.ANSB2 = 0;
    ANSELBbits.ANSB3 = 0;
    i2c_master_setup();
}
void i2c_imu_init(void){
    i2c_master_start();
    i2c_master_send(0xD6); //0b1101011 + 0write 11010110: D6
    i2c_master_send(0x10);  // Reg ADDRESS CTRL1_XL (10h)
    i2c_master_send(0x80); //1000 00 00
    i2c_master_stop();
    //set latch
   
    i2c_master_start();
    i2c_master_send(0xD6); //0b1101011 + 0write 11010110: D6
    i2c_master_send(0x11);  // Reg ADDRESS CTRL2_G (11h)
    i2c_master_send(0x80); //1000 00 00
    i2c_master_stop();
    
    i2c_master_start();
    i2c_master_send(0xD6); //0b1101011 + 0write 11010110: D6
    i2c_master_send(0x12);  // Reg ADDRESS CTRL3_C (12h)
    i2c_master_send(0x04); //0000 0100
    i2c_master_stop();
}
void LCD_drawChar(unsigned short x, unsigned short y, char word,unsigned short color,unsigned short background) {
    
    int i,j;
    for (i=0;i<5;i++)
    {
        for (j=0;j<8;j++)
        {
            if((ASCII[word - 0x20][i]>>j) % 2 != 0)//ascii && (0x01<<j)
            {
                LCD_drawPixel(x+i, y+j, color);        
            }
            else
            {
                LCD_drawPixel(x+i, y+j, background);
            }
        }
    }
}

void LCD_drawMessage(unsigned short x, unsigned short y, char message[20],unsigned short color,unsigned short background) 
{
    
    int i = 0; 
    while(message[i])
    { 
        if(x>123)
        {
            y = y+8;
            x = 0;
        }
        LCD_drawChar(x,y,message[i],color,background); 
        i++;
        x = x+6;
        
    }
}
void I2C_read_multiple(char reg_address, unsigned char data[20], char length)
{
    int i;
    i2c_master_start();
    i2c_master_send(0xD6);
    i2c_master_send(reg_address);
    i2c_master_restart();
    i2c_master_send(0xD7);
    for (i=0;i<length-1;i++)
    {
        data[i]=i2c_master_recv();
        i2c_master_ack(0);
    }
    data[length-1]=i2c_master_recv();
    i2c_master_ack(1);
    i2c_master_stop();
}
int main(void) {
    
    char message[20];
    unsigned short x,y;
    char word;
    unsigned char input;
    unsigned char data[20];
    short temp,acc_x,acc_y,acc_z,gyro_x,gyro_y,gyro_z;
    float acc_xf,acc_yf,acc_zf,gyro_xf,gyro_yf,gyro_zf;
    short data_16[10];
    int i,j;
    TRISAbits.TRISA4 = 0;       // make A4(PORT 12) as output
    LATAbits.LATA4 = 1;
    
    i2c_init();
    i2c_imu_init();
    
    SPI1_init();
    LCD_init();
    LCD_clearScreen(CYAN);//BLACK WHITE BLUE RED GREEN CYAN MAGENTA YELLOW
    
    i2c_master_start();
    i2c_master_send(0xD6);
    i2c_master_send(0x0F);
    i2c_master_restart();
    i2c_master_send(0xD7);
    input=i2c_master_recv();
    i2c_master_ack(1);
    i2c_master_stop();
    
    for(i=0;i<8;i++)
    {
        message[i] = (input>>i) && 0x01; 
    }
    
    while(1)
    {
   
    {;}
    I2C_read_multiple(0x20, data, 14);

    for(i=0;i<10;i++)
    {
        data_16[i] = (data[i*2+1]<<8)|(data[i*2]);  
    }
    
    data_16[0] = (data[1]<<8)|(data[0]);
 
    temp = -data_16[0];
    acc_x = data_16[4];
    acc_y = data_16[5];
    acc_z = data_16[6];
    acc_xf = acc_x / 32768.0 * -2;
    acc_yf = acc_y / 32768.0 * -2;
    acc_zf = acc_z / 32768.0 * -2;
    gyro_x = data_16[1];
    gyro_y = data_16[2];
    gyro_z = data_16[3];
    gyro_xf = gyro_x / 32768.0 * -245;
    gyro_yf = gyro_y / 32768.0 * -245;
    gyro_zf = gyro_z / 32768.0 * -245;
    
    
    
    //LCD_clearScreen(CYAN);//BLACK WHITE BLUE RED GREEN CYAN MAGENTA YELLOW
    x = 1;
    y = 10;
    sprintf(message,"temp = %d",temp);
    LCD_drawMessage( x, y, message,RED,CYAN);
    
    x = 1;
    y = 20;
    sprintf(message,"gyro_x = %4.2f",gyro_xf);
    LCD_drawMessage( x, y, message,RED,CYAN);
    
    x = 1;
    y = 30;
    sprintf(message,"gyro_y = %4.2f",gyro_yf);
    LCD_drawMessage( x, y, message,RED,CYAN);
    
    x = 1;
    y = 40;
    sprintf(message,"gyr0_z = %4.2f",gyro_zf);
    LCD_drawMessage( x, y, message,RED,CYAN);
    
    x = 1;
    y = 50;
    sprintf(message,"acc_x = %4.3f g",acc_xf);
    LCD_drawMessage( x, y, message,RED,CYAN);
    
    x = 1;
    y = 60;
    sprintf(message,"acc_y = %4.3f g",acc_yf);
    LCD_drawMessage( x, y, message,RED,CYAN);
    
    x = 1;
    y = 70;
    sprintf(message,"acc_z = %4.3f g",acc_zf);
    LCD_drawMessage( x, y, message,RED,CYAN); 
    
    }
    
    return 0;
}