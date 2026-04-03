/*
 * I2C_driver.h
 *
 *  Created on: Dec 12, 2025
 *      Author: Karol
 */

#ifndef INC_I2C_DRIVER_H_
#define INC_I2C_DRIVER_H_

#include "main.h"
#define MAX_PRE 10
#define MAX_POST 10
#define MAX_FRAME_LENGHT 32
#define CURRENT_PRE(i) (Pre_post_send->table_pre[i])
#define CURRENT_POST(i) (Pre_post_send->table_post[i])



typedef struct{							// Struktura zawierająca dane poszczególnej ramki

	I2C_HandleTypeDef* hi2c;
	uint16_t addres;					// Pełny adres tz. adres urządzenia + bit W/R
	uint8_t data[MAX_FRAME_LENGHT]; 	// I2C nie ma ograniczenia ale bez przesady
	uint8_t size_data;					// Rozmiar tablicy
	uint32_t timeout;					//
	uint8_t delay;						// Opóźnienie np. w AM2320 należy poczekać 2ms na pomiar

}I2C_frame;

typedef struct{							// Struktura umożliwiająca wysłanie ramki przed ramką właściwą np. ramka WAKE UP, ramka zrób coś, ramka SLEEP
	I2C_frame table_pre[MAX_PRE];		// tablica ramek do wysyłki przed ramką główną
	uint8_t size_pre;					// Rozmiar table_pre
	I2C_frame table_post[MAX_POST];		// tablica ramek do wysyłki po ramce głównej
	uint8_t size_post;					// Rozmiar table_post
}I2C_pre_post_frame;


HAL_StatusTypeDef I2C_Transmit_message(I2C_frame* Rx_frame, I2C_pre_post_frame* Pre_post_send);
HAL_StatusTypeDef I2C_Receive_message(I2C_frame* Tx_frame, I2C_pre_post_frame* Pre_post_send);


#endif /* INC_I2C_DRIVER_H_ */
