// ELEC3850
// S2 2014
//
// EV Data System
//
// Group 13
//
// Chris Wallis		3006650
// Matt Tucker
// Oliver Stirling
// Jarrod Horne 
//  
// Main function
//
 
 
#include <math.h>
#include "adxl345/adxl345.h"
#include "i2cmaster.h"
#include "main.h"
#include "iom128.h"
#include <util/twi.h>

// Initialisation functions


char init_SD()
{
	return 0;
}

char init_accel()
{
	return 0;
}

char init_GPS()
{
	return 0;
}

char init_CAN()
{
	init_spi();
	init_shield();
	init_controller();
	return 0;
}


void init_spi()
{
		SPCR = (1<<SPE)|(1<<MSTR)|1|2; /* Master mode, SPI enabled, clock on slowest speed */
		DDRB = (1<<PB0)|(1<<PB1)|(1<<PB2); /* Ports set to output (MOSI, SS, CLK) */

}

void init_shield()
{
	
}


void init_controller()
{
	
}

char init_LCD()
{
	// configure the microprocessor pins for the data lines
	lcd_D7_ddr |= (1<<lcd_D7_bit);                  // 4 data lines - output
	lcd_D6_ddr |= (1<<lcd_D6_bit);
	lcd_D5_ddr |= (1<<lcd_D5_bit);
	lcd_D4_ddr |= (1<<lcd_D4_bit);

	// configure the microprocessor pins for the control lines
	lcd_E_ddr |= (1<<lcd_E_bit);                    // E line - output
	lcd_RS_ddr |= (1<<lcd_RS_bit);                  // RS line - output

	// initialize the LCD controller as determined by the defines (LCD instructions)
	lcd_init_4d();                                  // initialize the LCD display for a 4-bit interface
	return 0;
}

char init()
{
//	init_LCD();
	
	board_init();
	if(init_SD()) return 1;
	if(init_accel()) return 2;
	if(init_GPS()) return 3;
	if(init_CAN()) return 4;
	
	USARTInit(25);
	// configure the microprocessor pins for the data lines
                              // initialize the LCD display for a 4-bit interface

	return 0;
}


// Bluetooth data sending function
char bt_send_data(char iBT, int dataBT)
{
	return 0;
}

// SD data saving
char sd_save_data(char iSD, int dataSD)
{
	return 0;
}

// Initialise USART
void USARTInit(uint16_t ubrr_value)
{

   //Set Baud rate

   UBRR0L = ubrr_value;
   UBRR0H = (ubrr_value>>8);

   /*Set Frame Format


   >> Asynchronous mode
   >> No Parity
   >> 1 StopBit

   >> char size 8

   */

   UCSR0C=(3<<UCSZ0);


   //Enable The receiver and transmitter

   UCSR0B=(1<<RXEN)|(1<<TXEN);


}


//This function is used to read the available data
//from USART. This function will wait untill data is
//available.
char USARTReadChar()
{
   //Wait untill a data is available

   while(!(UCSR0A & (1<<RXC)))
   {
      //Do nothing
   }

   //Now USART has got data from host
   //and is available is buffer

   return UDR0;
}


//This fuction writes the given "data" to
//the USART which then transmit it via TX line
void USARTWriteChar(char data)
{
   //Wait untill the transmitter is ready

   while(!(UCSR0A & (1<<UDRE)))
   {
      //Do nothing
   }

   //Now write the data to USART buffer

   UDR0=data;
}


// Function to get data from sensor
char get_data (char iGD, char* displayDataGD, char* dataGD)
{
	*dataGD = USARTReadChar();
	sprintf(displayDataGD, "Data = %x", *dataGD);
	return 0;
}

void i2c_init(void);
unsigned char i2c_start(unsigned char address);
unsigned char i2c_start_no_address(void);
unsigned char i2c_readAck(void);
unsigned char i2c_readAck_test(void);
unsigned char i2c_write(unsigned char data);
unsigned char i2c_rep_start(unsigned char address);
unsigned char i2c_readNak(void);
void i2c_stop(void);

void display_data(char checkDE);

int main (void)
{
	// Insert system clock initialization code here (sysclk_init()).
	//DDRA = 0xFF;
	//DDRD = 0xFF;
	//PORTA = 0xFF;
	
	// Struct for get_data_ functions
	struct functs {
		char iF;
		int (*get_data_funct)(char* data, char* displayData);
		} functions[] = {
			{0, get_data_GPS}, {1, get_data_accel}, {2, get_data_speed}, {3, get_data_rpm}, {4, get_data_voltage}, {5, get_data_current}, {NULL, NULL} 
				};
	
	// Variables for data
	char displayData[] = "Sample Data = 12";
	char data;
	char check = 0;


	// Initialisation

	init_LCD();
	_delay_ms(1);
	lcd_write_string_4d("Initialising");
	_delay_ms(1000);
	check = init();
	
	

	if (check == 0)
	{
		lcd_write_instruction_4d(lcd_Clear);
		_delay_ms(4);
		lcd_write_string_4d("Initialised");
		_delay_ms(1000);	
	} else
	{
			display_error(check);
	}
	
	_delay_us(1000);
	lcd_write_instruction_4d(lcd_Clear);
	_delay_ms(4);
	lcd_write_string_4d(program_version);
	_delay_ms(1000);
	i2c_init();
	DDRA = 0xFF;
	
	// Cycle through sensors and display, save and send data
	while(1)
	{
		
		
		i2c_start(0xA6); //Start, slave address + write, ack
		i2c_write(0x31); //Data Format register
		i2c_readAck(); //ack
		TWDR = 0x01; //Put into +- 4G range
		i2c_readAck(); //ack
		i2c_stop();
		
		i2c_start(0xA6); //Start, slave address + write, ack
		i2c_write(0x2D); //Power Control register
		i2c_readAck(); //ack
		TWDR = 0x08; //Measurement mode
		i2c_readAck(); //ack
		i2c_stop();
		
		i2c_start(0xA6); //Start, slave address + write, ack
		i2c_write(0x32); //X Axis
		i2c_readNak(); //Nak
		i2c_start(0xA7);
		i2c_readNak();
		i2c_stop();
		char DATAX0 = TWDR;
		display_data(TWDR);
		i2c_start(0xA6); //Start, slave address + write, ack
		i2c_write(0x33); //X Axis
		i2c_readNak(); //Nak
		i2c_start(0xA7);
		i2c_readNak();
		char DATAX1 = TWDR;
		i2c_stop();
		//DATAX1 = TWDR;
		//display_data(TWDR);
		
		//i2c_start_no_address();
		//i2c_start(0x3B);
		//display_data(i2c_readAck_test());
		//display_data(TWDR);
		//i2c_stop();
		PORTA = 0x00;
		_delay_ms(200);
		
// //		for (char i = 0; functions[i].iF != NULL; i++)
// //		{
// //			check = functions[i].get_data_funct(&data, displayData);
// 			check = get_data(0, displayData, &data);
// 			lcd_write_instruction_4d(lcd_Clear);
// 			_delay_ms(4);
// 
// 			// display data from sensor
// 			lcd_write_string_4d(displayData);
// 			_delay_ms(4);
// 			
// 			// If error - display error code
// 			if(check) display_error(check);
// 			
// 			// Save data to SD card
// //			check = sd_save_data(i, data);
// 			_delay_ms(4);
// 			if(check) display_error(check);
// 			
// 			// Send data via bluetooth
// 	//		check = bt_send_data(i, data);
// 			_delay_ms(4);
// 			if(check) display_error(check);
// 			_delay_ms(1000);
//		}
	}
	
}

//Initialise I2C

/*************************************************************************
 Initialization of the I2C bus interface. Need to be called only once
*************************************************************************/
void i2c_init(void)
{
  /* initialize TWI clock: 100 kHz clock, TWPS = 0 => prescaler = 1 */
  
  TWSR = 0;                         /* no prescaler */
  TWBR = ((F_CPU/SCL_CLOCK)-16)/2;  /* must be > 10 for stable operation */
  
}/* i2c_init */

/*************************************************************************	
  Issues a start condition and sends address and transfer direction.
  return 0 = device accessible, 1= failed to access device
*************************************************************************/
unsigned char i2c_start(unsigned char address)
{
    uint8_t   twst;

	// send START condition
	TWCR = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN);

	// wait until transmission completed
	while(!(TWCR & (1<<TWINT)));
	
	// check value of TWI Status Register. Mask prescaler bits.
	twst = (TWSR & 0xF8);
	if ( (twst != TW_START) && (twst != TW_REP_START)) return 1;
	
	// send device address
	TWDR = address;
	TWCR = (1<<TWINT) | (1<<TWEN);
	
	// wait until transmission completed and ACK/NACK has been received
	while(!(TWCR & (1<<TWINT)));
	
	// check value of TWI Status Register. Mask prescalar bits.
	twst = TW_STATUS & 0xF8;
	if ( (twst != TW_MT_SLA_ACK) && (twst != TW_MR_SLA_ACK) ) return 1;
	return 0;
	

}/* i2c_start */


/*************************************************************************	
  Issues a start condition
*************************************************************************/
unsigned char i2c_start_no_address(void)
{
    uint8_t   twst;

	// send START condition
	TWCR = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN);

	// wait until transmission completed
	while(!(TWCR & (1<<TWINT)));
	
	// check value of TWI Status Register. Mask prescaler bits.
	twst = (TWSR & 0xF8);
	if ( (twst != TW_START) && (twst != TW_REP_START)) return 1;
	return 0;
	

}/* i2c_start_no_address */

/*************************************************************************
 Issues a repeated start condition and sends address and transfer direction 

 Input:   address and transfer direction of I2C device
 
 Return:  0 device accessible
          1 failed to access device
*************************************************************************/

unsigned char i2c_rep_start(unsigned char address)
{
	return i2c_start( address );

	}/* i2c_rep_start */

/*************************************************************************
  Send one byte to I2C device
  
  Input:    byte to be transfered
  Return:   0 write successful 
            1 write failed
*************************************************************************/
unsigned char i2c_write(unsigned char data)
{
	uint8_t   twst;
	
	// send data to the previously addressed device
	TWDR = data;
	TWCR = (1<<TWINT) | (1<<TWEN);

	// wait until transmission completed
	while(!(TWCR & (1<<TWINT)));

	// check value of TWI Status Register. Mask prescaler bits
	twst = TW_STATUS & 0xF8;
	if( twst != TW_MT_DATA_ACK) return 1;
	return 0;

	}/* i2c_write */

/*************************************************************************
 Read one byte from the I2C device, request more data from device 
 
 Return:  byte read from I2C device
*************************************************************************/
unsigned char i2c_readAck(void)
{
	TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWEA);
	while(!(TWCR & (1<<TWINT)));    // getting stuck in here. Won't get stuck if ! condition removed.
    return TWDR;

}/* i2c_readAck */

unsigned char i2c_readAck_test(void)
{
	TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWEA);
	while(!(TWCR & (1<<TWINT)));    // getting stuck in here. Won't get stuck if ! condition removed.
	return TWDR;

	}/* i2c_readAck_test */


/*************************************************************************
 Read one byte from the I2C device, read is followed by a stop condition 
 
 Return:  byte read from I2C device
*************************************************************************/
unsigned char i2c_readNak(void)
{
	TWCR = (1<<TWINT) | (1<<TWEN);
	while(!(TWCR & (1<<TWINT)));
	
    return TWDR;

}/* i2c_readNak */

/*************************************************************************
 Terminates the data transfer and releases the I2C bus
*************************************************************************/
void i2c_stop(void)
{
    /* send stop condition */
	TWCR = (1<<TWINT)|(1<<TWEN)|(1<<TWSTO);
	
	// wait until stop condition is executed and bus released
	while(!(TWCR & (1<<TWSTO)));

}/* i2c_stop */

void display(char* stringD)
{
	lcd_write_instruction_4d(lcd_Clear);
	_delay_ms(4);
	lcd_write_string_4d(stringD);
	_delay_ms(4);
}

// Display error on 2nd line of LCD
void display_error(char checkDE)
{
	char displayData[20];
	lcd_write_instruction_4d(lcd_Clear);
	_delay_ms(4);
	sprintf(displayData, "Error %x", checkDE);
	lcd_write_string_4d(displayData);
	_delay_ms(4);
}

// Display data on 2nd line of LCD
void display_data(char checkDE)
{
	char displayData[20];
	lcd_write_instruction_4d(lcd_Clear);
	_delay_ms(4);
	sprintf(displayData, "Data (hex)= %x", checkDE);
	lcd_write_string_4d(displayData);
	_delay_ms(4);
}


char get_data_GPS(char* data, char* displayData) 
{
	return 0;	
}

char get_data_accel(char* data, char* displayData)
{
	return 0;
}



char get_data_speed(char* data, char* displayData)
{
	return 0;
}

char get_data_rpm(char* data, char* displayData)
{
	return 0;
}

char get_data_voltage(char* data, char* displayData)
{
	return 0;
}

char get_data_current(char* data, char* displayData)
{
	return 0;
}



void lcd_init_4d(void)
{
// Power-up delay
    _delay_ms(1000);                                 // initial 40 mSec delay
// Set up the RS and E lines for the 'lcd_write_4' subroutine.
    lcd_RS_port &= ~(1<<lcd_RS_bit);                // select the Instruction Register (RS low)
    lcd_E_port &= ~(1<<lcd_E_bit);                  // make sure E is initially low

// Reset the LCD controller
    lcd_write_4(lcd_FunctionReset);                 // first part of reset sequence
    _delay_ms(10);                                  // 4.1 mS delay (min)

    lcd_write_4(lcd_FunctionReset);                 // second part of reset sequence
    _delay_us(200);                                 // 100uS delay (min)

    lcd_write_4(lcd_FunctionReset);                 // third part of reset sequence
    _delay_us(200);                                 // this delay is omitted in the data sheet

 
    lcd_write_4(lcd_FunctionSet4bit);               // set 4-bit mode
    _delay_us(80);                                  // 40uS delay (min)

    lcd_write_instruction_4d(lcd_FunctionSet4bit);   // set mode, lines, and font
    _delay_us(80);                                  // 40uS delay (min)


// Display On/Off Control instruction
    lcd_write_instruction_4d(lcd_DisplayOff);        // turn display OFF
    _delay_us(80);                                  // 40uS delay (min)

// Clear Display instruction
    lcd_write_instruction_4d(lcd_Clear);             // clear display RAM
    _delay_ms(4);                                   // 1.64 mS delay (min)

// ; Entry Mode Set instruction
    lcd_write_instruction_4d(lcd_EntryMode);         // set desired shift characteristics
    _delay_us(80);                                  // 40uS delay (min)

 
// Display On/Off Control instruction
    lcd_write_instruction_4d(lcd_DisplayOn);         // turn the display ON
    _delay_us(80);                                  // 40uS delay (min)
}

/*...........................................................................
  Name:     lcd_write_string_4d
; Purpose:  display a string of characters on the LCD
  Entry:    (theString) is the string to be displayed
  Exit:     no parameters
  Notes:    uses time delays rather than checking the busy flag
*/
void lcd_write_string_4d(uint8_t theString[])
{
    volatile int i = 0;                             // character counter*/
    while (theString[i] != 0)
    {
        lcd_write_character_4d(theString[i]);
        i++;
        _delay_us(80);                              // 40 uS delay (min)
    }
}

/*...........................................................................
  Name:     lcd_write_character_4d
  Purpose:  send a byte of information to the LCD data register
  Entry:    (theData) is the information to be sent to the data register
  Exit:     no parameters
  Notes:    does not deal with RW (busy flag is not implemented)
*/

void lcd_write_character_4d(uint8_t theData)
{
    lcd_RS_port |= (1<<lcd_RS_bit);                 // select the Data Register (RS high)
    lcd_E_port &= ~(1<<lcd_E_bit);                  // make sure E is initially low
    lcd_write_4(theData);                           // write the upper 4-bits of the data
    lcd_write_4(theData << 4);                      // write the lower 4-bits of the data
}

/*...........................................................................
  Name:     lcd_write_instruction_4d
  Purpose:  send a byte of information to the LCD instruction register
  Entry:    (theInstruction) is the information to be sent to the instruction register
  Exit:     no parameters
  Notes:    does not deal with RW (busy flag is not implemented)
*/
void lcd_write_instruction_4d(uint8_t theInstruction)
{
    lcd_RS_port &= ~(1<<lcd_RS_bit);                // select the Instruction Register (RS low)
    lcd_E_port &= ~(1<<lcd_E_bit);                  // make sure E is initially low
    lcd_write_4(theInstruction);                    // write the upper 4-bits of the data
    lcd_write_4(theInstruction << 4);               // write the lower 4-bits of the data
}


/*...........................................................................
  Name:     lcd_write_4
  Purpose:  send a byte of information to the LCD module
  Entry:    (theByte) is the information to be sent to the desired LCD register
            RS is configured for the desired LCD register
            E is low
            RW is low
  Exit:     no parameters
  Notes:    use either time delays or the busy flag
*/
void lcd_write_4(uint8_t theByte)
{
    lcd_D7_port &= ~(1<<lcd_D7_bit);                        // assume that data is '0'
    if (theByte & 1<<7) lcd_D7_port |= (1<<lcd_D7_bit);     // make data = '1' if necessary

    lcd_D6_port &= ~(1<<lcd_D6_bit);                        // repeat for each data bit
    if (theByte & 1<<6) lcd_D6_port |= (1<<lcd_D6_bit);

    lcd_D5_port &= ~(1<<lcd_D5_bit);
    if (theByte & 1<<5) lcd_D5_port |= (1<<lcd_D5_bit);

    lcd_D4_port &= ~(1<<lcd_D4_bit);
    if (theByte & 1<<4) lcd_D4_port |= (1<<lcd_D4_bit);

// write the data
                                                    // 'Address set-up time' (40 nS)
    lcd_E_port |= (1<<lcd_E_bit);                   // Enable pin high
    _delay_us(1);                                   // implement 'Data set-up time' (80 nS) and 'Enable pulse width' (230 nS)
    lcd_E_port &= ~(1<<lcd_E_bit);                  // Enable pin low
    _delay_us(1);                                   // implement 'Data hold time' (10 nS) and 'Enable cycle time' (500 nS)
}
