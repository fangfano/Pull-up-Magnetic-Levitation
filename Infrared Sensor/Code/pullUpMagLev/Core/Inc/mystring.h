/*
 * mystring.h
 *
 *  Created on: Jul 1, 2021
 *      Author: han
 */

#ifndef INC_MYSTRING_H_
#define INC_MYSTRING_H_

#include "stm32f1xx_hal.h"

void uitoa(uint8_t* dest,uint32_t num);
void myitoa(uint8_t* dest,int32_t num);
void ftoa(uint8_t* dest,float num,uint8_t bits);
uint8_t ftoa_c(uint8_t *dest, float num, uint8_t bits_i, uint8_t bits_d);

#endif /* INC_MYSTRING_H_ */
