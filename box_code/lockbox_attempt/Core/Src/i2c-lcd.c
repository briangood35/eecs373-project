/** Put this in the src folder **/

#include "i2c-lcd.h"
#include <stdint.h>
#include <stdbool.h>
#include "main.h"
extern I2C_HandleTypeDef hi2c1;  // change your handler here accordingly

#define MOTORLOGPORT GPIOG
#define MOTORLOG1 GPIO_PIN_0
#define MOTORLOG2 GPIO_PIN_1
extern ADC_HandleTypeDef hadc1;
extern bool open;
extern int IR_count;


#define SLAVE_ADDRESS_LCD (0x50) // 0x40 if ignoring r/w // for small LCD 0x4E // change this according to ur setup

extern uint32_t* tim_4_ccr2;

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
	HAL_Delay(100);
	lcd_send_cmd (0x41); //turn on display
	lcd_send_cmd (0x51); //clear display
	lcd_send_cmd (0x46); //home the curser

	//write password
	lcd_send_string("Password: ");

	lcd_send_cmd (0x47); //turn on underlying cursur
	lcd_send_cmd (0x4B);

}

void close_door (void)
{
	//close door
		lcd_send_string("CLOSING");
		HAL_GPIO_WritePin(MOTORLOGPORT, MOTORLOG1, 0);
		HAL_GPIO_WritePin(MOTORLOGPORT, MOTORLOG2, 1);
		HAL_Delay(3000);
		lcd_send_cmd (0x51); //clear display
		lcd_send_cmd (0x46); //home the curser
		lcd_send_string("Password: ");

		lcd_send_cmd (0x47); //turn on underlying cursur
		lcd_send_cmd (0x4B);
		open = false;

}

void read_IR (void)
{
	if(open == true){
		uint32_t ADC_VAL = 0;

		//check if open or closed
		HAL_ADC_Start(&hadc1);//start conversion
		HAL_ADC_PollForConversion(&hadc1, 0xFFFFFFFF);//wait for conversion to finish
		ADC_VAL = HAL_ADC_GetValue(&hadc1);//retrieve value

		if(ADC_VAL > 750){
			IR_count++;
		}
		else{IR_count = 0;}
		if(IR_count >= 10){
			close_door();
			IR_count = 0;
		}

//		char str[20]; // Make sure the buffer is large enough to hold the string representation of the integer
//		// Convert integer to string
//		sprintf(str, "%d", ADC_VAL);
//		lcd_send_string(str);
//		HAL_Delay(500);
//		lcd_send_cmd (0x51); //clear display


	}
}
void lcd_correct_pass (void)
{
	//write correct
	lcd_send_cmd_param(0x45, 0x40);
	lcd_send_string("Correct$$$");


	*tim_4_ccr2 =190 ;
	HAL_Delay(500);
	HAL_GPIO_WritePin(MOTORLOGPORT, MOTORLOG1, 1);
	HAL_GPIO_WritePin(MOTORLOGPORT, MOTORLOG2, 0);
	HAL_Delay(1000);
	*tim_4_ccr2 = 150;
	HAL_Delay(3000);
	open = true;

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
