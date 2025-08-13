/*
 * mystring.c
 *
 *  Created on: Jul 1, 2021
 *      Author: han
 */
#include "mystring.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "stm32f1xx_hal.h"

void uitoa(uint8_t *dest, uint32_t num) {
	uint8_t *tmp = malloc(10);
	int i, bits;

	if (num == 0) {
		dest[0] = '0';
		dest[1] = 0;
		free((void*) tmp);
		return;
	}

	for (i = 0; num > 0; i++) {
		tmp[i] = (num % 10) + '0';
		num /= 10;
	}
	bits = i;
	for (i = 0; i < bits; i++) {
		dest[i] = tmp[bits - i - 1];
	}
	dest[bits] = 0;
	free((void*) tmp);
}

void myitoa(uint8_t *dest, int32_t num) {
	if (num >= 0) {
		uitoa(dest, (uint32_t) num);
		return;
	} else {
		dest[0] = '-';
		uitoa(dest + 1, (uint32_t) (0 - num));
		return;
	}
}

void ftoa(uint8_t *dest, float num, uint8_t bits) {
	uint32_t integer;
	uint32_t decimal;
	uint8_t sign = 0;
	uint8_t i = 0;
	uint8_t *tmp2 = malloc(10);
	uint8_t len1, len2;
	if (num < 0) {
		sign = 1;
		num = 0 - num;
		dest[i] = '-';
		i++;
	}
	integer = (uint32_t) num;
	decimal = (uint32_t) ((num - integer) * pow(10, bits) + 0.5);

	if (sign) {
		uitoa(dest + 1, integer);
	} else {
		uitoa(dest, integer);
	}
	uitoa(tmp2, decimal);

	len1 = strlen((char*) dest);
	len2 = strlen((char*) tmp2);

	strcat((char*) dest, ".");
	for (i = 0; i < (bits - len2); i++) {
		dest[len1 + i + 1] = '0';
	}
	dest[len1 + i + 1] = 0;

	strcat((char*) dest, (char*) tmp2);
	free(tmp2);
}

/**************************************
 * 设置整数位数和小数位数，不够时整数部分空格补齐，小数部分0补齐
 * 注意
 * ！！！！！！   负号占一位   ！！！！！
 * 超出位数时整数部分忽略多出的高位，小数部分四舍五入
 * 如 ftoa_c(dest,1.2,2,2)；
 * 得到的变量dest为“ 1.20”
 * @param 所得数据的储存地址
 * @param 数字
 * @param 整数位数
 * @param 小数位数
 * @ret 如果整数部分溢出，返回1，否则返回0；
 * ***********************************************/
uint8_t ftoa_c(uint8_t *dest, float num, uint8_t bits_i, uint8_t bits_d) {
	uint8_t *tmp1;
	uint8_t i, j;
	tmp1 = malloc(10);
	ftoa(tmp1, num, bits_d);
	for (i = 0; i < 10; i++) {
		if (tmp1[i] == '.')
			break;
	}
	if (i == bits_i) {
		strcpy((char*)dest, (char*)tmp1);
		free((void*) tmp1);
		return 0;
	} else if (i > bits_i) {
		strcpy((char*)dest, (char*)(tmp1 + i - bits_i));
		free((void*) tmp1);
		return 1;
	} else {
		for (j = 0; j < bits_i - i; j++) {
			dest[j] = ' ';
		}
		strcpy((char*)(dest + bits_i - i),(char*) tmp1);
		free((void*) tmp1);
		return 0;
	}
}


