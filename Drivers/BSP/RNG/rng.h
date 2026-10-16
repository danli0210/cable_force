#ifndef __RNG_H
#define __RNG_H 

#include "./SYSTEM/sys/sys.h"

uint8_t rng_init(void);                         /* Initialize RNG */
uint32_t rng_get_random_num(void);              /* Get a random number */
int rng_get_random_range(int min, int max);     /* Get a random number within a specified range */

#endif
