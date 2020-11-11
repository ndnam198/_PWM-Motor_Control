/**
 * @file myMisc.h
 * @author Nam Nguyen (ndnam198@gmail.com)
 * @brief Miscellaneous implementations & functions that can generally used by any type of chips
 * @version 0.1
 * @date 2020-11-02
 * 
 * @copyright Copyright (c) 2020
 * 
 */
#ifndef __MY_MISC_H
#define __MY_MISC_H

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdlib.h>
#include "main.h"

/****************************************** GENERAL-DEFINE-BEGIN */

/* Compare a string to a destined string, return 1 if equal, else return 0 */
#define IS_MY_STRING(src, des) ((strcmp((char *)src, des)) == 0 ? 1 : 0)

/* Return the number of elements in an array */
#define NUMBER_OF_ELEMENT(array) (sizeof(array) / sizeof(array[0]))

/* Return a random number between min and max value */
#define RAND_U32(min, max) (ucRandomNumber(min, max))

/* Create a random number in range of [min,max] */
uint32_t ucRandomNumber(uint32_t min, uint32_t max);


#endif /* __MYLIB_H */
