/*
 * CFile1.c
 *
 * Created: 24/09/2014 4:42:44 PM
 *  Author: c3006_000
 */ 


// Program ID
uint8_t program_author[]   = "Chris Wallis";
uint8_t program_version[]  = "LCD TEST0.1";
uint8_t program_date[]     = "22 Sept 2014";

// Function Prototypes
void lcd_write_4(uint8_t);
void lcd_write_instruction_4d(uint8_t);
void lcd_write_character_4d(uint8_t);
void lcd_write_string_4d(uint8_t *);
void lcd_init_4d(void);


/******************************* End of Main Program Code ******************/

/*============================== 4-bit LCD Functions ======================*/
// Initialise the LCD

