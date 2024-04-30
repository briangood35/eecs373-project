/** Put this in the src folder **/

#include "i2c-lcd.h"
#include <stdint.h>
#include "main.h"
extern I2C_HandleTypeDef hi2c1;  // change your handler here accordingly
extern I2C_HandleTypeDef hsmbus2;
extern SPI_HandleTypeDef hspi1;
#define SLAVE_ADDRESS_LCD (0x50) // 0x40 if ignoring r/w // for small LCD 0x4E // change this according to ur setup
#define SLAVE_ADDRESS_CAM_R (0x61)
#define SLAVE_ADDRESS_CAM_W (0x60)

extern uint32_t* tim_4_ccr2;

void cam_setup (void){
	uint8_t data_t[1];
	data_t[0] = 0x11;
	HAL_I2C_Master_Transmit (&hsmbus2, SLAVE_ADDRESS_CAM_W,(uint8_t *) data_t, 2, 100);
}

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
	HAL_Delay(200);
	lcd_send_string("Password: ");

	lcd_send_cmd (0x47); //turn on underlying cursur
	lcd_send_cmd (0x4B);
}

void close_door (void)
{
	//close door
		lcd_send_string("CLOSING");
		*tim_4_ccr2 = 100;
		HAL_Delay(1000);
		lcd_send_cmd (0x51); //clear display
		lcd_send_cmd (0x46); //home the curser
		lcd_send_string("Password: ");

		lcd_send_cmd (0x47); //turn on underlying cursur
		lcd_send_cmd (0x4B);
		//open = false;

}

void cam_take_pic(void){
	uint32_t IMAGE_SIZE = (60 * 80 * 3);

	uint8_t data_t[2];
//	data_t[0] = 0x12;//system reset
//	data_t[1] = 0x80;
//	HAL_I2C_Master_Transmit (&hsmbus2, SLAVE_ADDRESS_CAM_W,(uint8_t *) data_t, 2, 100);


	data_t[0] = 0xE0;
	data_t[1] = 0x10;
	HAL_I2C_Master_Transmit (&hsmbus2, SLAVE_ADDRESS_CAM_W,(uint8_t *) data_t, 2, 100);

	data_t[0] = 0xDA;
	data_t[1] = 0x10;
	HAL_I2C_Master_Transmit (&hsmbus2, SLAVE_ADDRESS_CAM_W,(uint8_t *) data_t, 2, 100);

//	data_t[0] = 0x0C;
//	data_t[1] = 0x01;
//	HAL_I2C_Master_Transmit (&hsmbus2, SLAVE_ADDRESS_CAM_W,(uint8_t *) data_t, 2, 100);

	uint8_t command = 0x84;

	// Send command to capture image
	//HAL_GPIO_WritePin(CAMERA_CS_PORT, CAMERA_CS_PIN, GPIO_PIN_RESET); // Activate camera CS
	HAL_SPI_Transmit(&hspi1, &command, 1, HAL_MAX_DELAY); // Send command over SPI
    //HAL_GPIO_WritePin(CAMERA_CS_PORT, CAMERA_CS_PIN, GPIO_PIN_SET); // Deactivate camera CS

    // Wait for image capture (adjust the delay as needed)
    HAL_Delay(500);

    // Receive image data over SPI
    uint8_t image_data[IMAGE_SIZE]; // Define an array to store image data
    //HAL_GPIO_WritePin(CAMERA_CS_PORT, CAMERA_CS_PIN, GPIO_PIN_RESET); // Activate camera CS
    HAL_SPI_Receive(&hspi1, image_data, IMAGE_SIZE, HAL_MAX_DELAY); // Receive image data over SPI
    //HAL_GPIO_WritePin(CAMERA_CS_PORT, CAMERA_CS_PIN, GPIO_PIN_SET);

    HAL_Delay(500);







//	uint8_t data_t[1];
//	data_t[0] = 0x10;
//	//HAL_I2C_Master_Transmit (&hsmbus2, SLAVE_ADDRESS_CAM_W,(uint8_t *) data_t, 2, 100);
//
//	//HAL_SPI_Receive(&hspi1, data_t, 2, HAL_MAX_DELAY);
//
//	HAL_I2C_Master_Transmit(&hsmbus2, SLAVE_ADDRESS_CAM_W, data_t, 1, 100);
//	HAL_Delay(500);
//
//	uint8_t image_data[(60 * 80 * 3)]; // Define an array to store image data
//
//	//HAL_SPI_Receive(&hspi1, image_data, (60 * 80 * 3), HAL_MAX_DELAY);
//
//	HAL_I2C_Master_Receive(&hsmbus2, SLAVE_ADDRESS_CAM_R, image_data, sizeof(image_data), HAL_MAX_DELAY);
//
//	HAL_Delay(500);

}

void lcd_correct_pass (void)
{


	//write correct
	cam_take_pic();
	lcd_send_cmd_param(0x45, 0x40);
	lcd_send_string("Correct$$$");

	//open lock funciton
	*tim_4_ccr2 = 240;
	HAL_Delay(3000);




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
