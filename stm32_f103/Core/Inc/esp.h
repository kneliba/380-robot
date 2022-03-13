#ifndef ESP_H
#define ESP_H
#include "main.h"

void ESP_send(I2C_HandleTypeDef *hi2c1, uint8_t register_address, uint16_t size, uint8_t data);
void ESP_receive(I2C_HandleTypeDef *hi2c1, uint8_t register_address, uint16_t size, uint8_t *Receive);

#endif /* ESP_H */
