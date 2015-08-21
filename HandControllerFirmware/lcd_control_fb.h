#define LCD_line PORTA

#define LCD_RS 0
#define LCD_RW 1
#define LCD_EN 2
#define LCD_DB7 7
#define LCD_DB6 6
#define LCD_DB5 5
#define LCD_DB4 4

#define DB_MASK 0xF0
#define EN_MASK 0x04

#define LCD_DUR_US 5
 
void LCD_busy();
void LCD_initialize();
void display_String(char* word);
void LCD_changeline(int num);

