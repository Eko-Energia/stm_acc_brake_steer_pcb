/*
 * I2C_driver.c
 *
 *  Created on: Dec 12, 2025
 *      Author: Karol
 */
#include "I2C_driver.h"

HAL_StatusTypeDef I2C_Transmit_message(I2C_frame* Tx_frame, I2C_pre_post_frame* Pre_post_send) // Funkcja do wysyłki danych po I2C
/*
 *
 * ARGS:
	 * Tx_frame - struktura z informacją o ramce głównej
	 * Pre_post_send - struktura z ramkami do wysyłki po i przed ramką główną
 * RETURN:
 	 * HAL_OK - pomyślnie wykonano operacje
 	 * HAL_ERROR - wystąpił błąd -> wywołąnie asser_failed()
 *
 */

{

	if (Tx_frame->addres <= 0x7F)							// Adres wraz z bitem Write/Read ma dokładnie 8 bitów
	{
		assert_failed(__FILE__, __LINE__);
	}

	for (uint8_t i = 0; i < Pre_post_send->size_pre; i++)	// Informacja przed ramką główną
	{
		HAL_I2C_Master_Transmit(CURRENT_PRE(i).hi2c, CURRENT_PRE(i).addres, NULL, 0, CURRENT_PRE(i).timeout);
		HAL_Delay(CURRENT_PRE(i).delay);						//OPCJONALNY ALE DLA AM2320 MUSI BYC
	}

	if (HAL_I2C_Master_Transmit(Tx_frame->hi2c, Tx_frame->addres, Tx_frame->data , Tx_frame->size_data, Tx_frame->timeout) != HAL_OK )
	{
		assert_failed(__FILE__, __LINE__);					// Ramka główna
	}

	for (uint8_t i = 0; i < Pre_post_send->size_post; i++)	// Informacja po ramce głównej
	{
		HAL_I2C_Master_Transmit(CURRENT_POST(i).hi2c, CURRENT_POST(i).addres, CURRENT_POST(i).data, CURRENT_POST(i).size_data, CURRENT_POST(i).timeout);
		HAL_Delay(CURRENT_POST(i).delay);					//OPCJONALNY ALE DLA AM2320 MUSI BYC
	}


	return HAL_OK;
}



HAL_StatusTypeDef I2C_Receive_message(I2C_frame* Rx_frame, I2C_pre_post_frame* Pre_post_send)
/*
 *
 * ARGS:
	 * Rx_frame - struktura z informacją o ramce głównej
	 * Pre_post_send - struktura z ramkami do wysyłki po i przed ramką główną
 * RETURN:
 	 * HAL_OK - pomyślnie wykonano operacje
 	 * HAL_ERROR - wystąpił błąd -> wywołąnie asser_failed()
 *
 */
{
	HAL_Delay(Rx_frame->delay);
	if (Rx_frame->addres <= 0x7F)// adres wraz z bitem Write/Read ma dokładnie 8 bitów
	{
		assert_failed(__FILE__, __LINE__);
	}

	for (uint8_t i = 0; i < Pre_post_send->size_pre; i++)		// Informacja przed ramą główną
	{
		HAL_I2C_Master_Transmit(CURRENT_PRE(i).hi2c, CURRENT_PRE(i).addres, NULL, 0, CURRENT_PRE(i).timeout);
		HAL_Delay(CURRENT_PRE(i).delay);						//OPCJONALNY ALE DLA AM2320 MUSI BYC
	}

	if (HAL_I2C_Master_Receive(Rx_frame->hi2c, Rx_frame->addres, Rx_frame->data, Rx_frame->size_data, Rx_frame->timeout) != HAL_OK )
	{
		assert_failed(__FILE__, __LINE__);
		return HAL_ERROR;										// Ramka główna
	}

	for (uint8_t i = 0; i < Pre_post_send->size_post; i++)		// Informacja po ramce głównej
	{
		HAL_I2C_Master_Transmit(CURRENT_POST(i).hi2c, CURRENT_POST(i).addres, CURRENT_POST(i).data, CURRENT_POST(i).size_data, CURRENT_POST(i).timeout);
		HAL_Delay(CURRENT_POST(i).delay);						//OPCJONALNY ALE DLA AM2320 MUSI BYC
	}

	return HAL_OK;
}
