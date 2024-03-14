/** Put this in the src folder **/

#include "i2c-lcd.h"
#include <stdint.h>
#include "main.h"
extern I2C_HandleTypeDef hi2c1;  // change your handler here accordingly

#define SLAVE_ADDRESS_LCD (0x50) // 0x40 if ignoring r/w // for small LCD 0x4E // change this according to ur setup

void lcd_send_cmd (char cmd) //everything needs to start with FE then the comand. Dont know why its a char
{
	uint8_t data_t[2];
	data_t[0] = 0xFE;
	data_t[1] = cmd;
	HAL_I2C_Master_Transmit (&hi2c1, SLAVE_ADDRESS_LCD,(uint8_t *) data_t, 2, 100);


}

void lcd_send_cmd_param (char cmd, char param) //everything needs to start with FE then the comand. Dont know why its a char
{
	uint8_t data_t[3];
	data_t[0] = 0xFE;
	data_t[1] = cmd;
	data_t[2] = param;
	HAL_I2C_Master_Transmit (&hi2c1, SLAVE_ADDRESS_LCD,(uint8_t *) data_t, 3, 100);


}

void lcd_send_data (char data_in)
{
	uint8_t data_t[1];
	//data_t[1] = 0xFE;
	data_t[0] = data_in;
	HAL_I2C_Master_Transmit (&hi2c1, SLAVE_ADDRESS_LCD,(uint8_t *) data_t, 1, 100);
}


void lcd_init (void)
{
	lcd_send_cmd (0x41); //turn on display
	lcd_send_cmd (0x51); //clear display
	lcd_send_cmd (0x46); //home the curser

	//write password
	lcd_send_string("Password: ");

	lcd_send_cmd (0x47); //turn on underlying cursur
	lcd_send_cmd (0x4B);
}

void lcd_correct_pass (void)
{
	//write correct
	lcd_send_cmd_param(0x45, 0x40);
	lcd_send_string("Correct$$$");

	//open lock funciton
	HAL_Delay(2000);

}

void lcd_incorrect_pass (void)
{
	//write correct
		lcd_send_cmd_param(0x45, 0x40);
		lcd_send_string("Incorrect :(");

		//open lock funciton
		HAL_Delay(2000);
}

void lcd_nice(void){
	lcd_send_cmd_param(0x45, 0x40);
	lcd_send_string("Nice! :D");
	HAL_Delay(2000);


}

void lcd_send_string (char *str) //just going to do it manually
{
	while (*str) lcd_send_data (*str++);
}
