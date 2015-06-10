/*
 * IncFile1.h
 *
 * Created: 25/09/2014 12:11:13 AM
 *  Author: c3006_000
 */ 


#ifndef MAIN_H_
#define MAIN_H_

// Definitions for LCD
#define lcd_D7_port     PORTG 
#define lcd_D7_bit      PG3
#define lcd_D7_ddr      DDRG

#define lcd_D6_port     PORTG  
#define lcd_D6_bit      PG2
#define lcd_D6_ddr      DDRG

#define lcd_D5_port     PORTG      
#define lcd_D5_bit      PG1
#define lcd_D5_ddr      DDRG

#define lcd_D4_port     PORTG       
#define lcd_D4_bit      PG0
#define lcd_D4_ddr      DDRG

#define lcd_E_port      PORTD        
#define lcd_E_bit       PD7
#define lcd_E_ddr       DDRD

#define lcd_RS_port     PORTG               
#define lcd_RS_bit      PG4
#define lcd_RS_ddr      DDRG


#define lcd_Clear           0b00000001          // replace all characters with ASCII 'space'
#define lcd_Home            0b00000010          // return cursor to first position on first line
#define lcd_EntryMode       0b00000110          // shift cursor from left to right on read/write
#define lcd_DisplayOff      0b00001000          // turn display off
#define lcd_DisplayOn       0b00001100          // display on, cursor off, don't blink character
#define lcd_FunctionReset   0b00110000          // reset the LCD
#define lcd_FunctionSet4bit 0b00101000          // 4-bit data, 2-line display, 5 x 7 font
#define lcd_SetCursor       0b10000000          // set cursor position

// LCD module information
#define lcd_LineOne     0x00                    // start of line 1
#define lcd_LineTwo     0x40                    // start of line 2
//#define   lcd_LineThree   0x14                  // start of line 3 (20x4)
//#define   lcd_lineFour    0x54                  // start of line 4 (20x4)
//#define   lcd_LineThree   0x10                  // start of line 3 (16x4)
//#define   lcd_lineFour    0x50                  // start of line 4 (16x4)


/*
 * Include header files for all drivers that have been imported from
 * Atmel Software Framework (ASF).
 */
#include <asf.h>
#define F_CPU 16000000UL
#define SCL_CLOCK  100000L
#define NO_OF_SENSORS 1
#include <avr/io.h>
#include <util/delay.h>
#include "LCD.c"
//#include <inttypes.h>
#include <stdio.h>


// Function Prototypes
char init_SD(void);
char init_accel(void);
char init_GPS(void);
char init_CAN(void);
char init(void);
void USARTInit(uint16_t ubrr_value);
char USARTReadChar(void);
void USARTWriteChar(char data);
char get_data (char iGD, char* displayDataGD, char* dataGD);
void display_error(char checkDE);
void init_spi(void);
void init_shield(void);
void init_controller(void);
char get_data_GPS(char* data, char* displayData);
char get_data_accel(char* data, char* displayData);
char get_data_speed(char* data, char* displayData);
char get_data_rpm(char* data, char* displayData);
char get_data_voltage(char* data, char* displayData);
char get_data_current(char* data, char* displayData);
char init_LCD(void);
char bt_send_data(char iBT, int dataBT);
char sd_save_data(char iSD, int dataSD);




#endif /* MAIN_H_ */