#ifndef __KEY_H
#define __KEY_H

#include "./SYSTEM/sys/sys.h"

/******************************************************************************************/
/* Pin Definitions */

#define KEY0_GPIO_PORT                  GPIOE
#define KEY0_GPIO_PIN                   GPIO_PIN_4
#define KEY0_GPIO_CLK_ENABLE()          do{ __HAL_RCC_GPIOE_CLK_ENABLE(); }while(0)   /* GPIOE clock enable */

#define KEY1_GPIO_PORT                  GPIOE
#define KEY1_GPIO_PIN                   GPIO_PIN_3
#define KEY1_GPIO_CLK_ENABLE()          do{ __HAL_RCC_GPIOE_CLK_ENABLE(); }while(0)   /* GPIOE clock enable */

#define KEY2_GPIO_PORT                  GPIOE
#define KEY2_GPIO_PIN                   GPIO_PIN_2
#define KEY2_GPIO_CLK_ENABLE()          do{ __HAL_RCC_GPIOE_CLK_ENABLE(); }while(0)   /* GPIOE clock enable */

#define WKUP_GPIO_PORT                  GPIOA
#define WKUP_GPIO_PIN                   GPIO_PIN_0
#define WKUP_GPIO_CLK_ENABLE()          do{ __HAL_RCC_GPIOA_CLK_ENABLE(); }while(0)   /* GPIOA clock enable */

/******************************************************************************************/
/* Key Reading Macros */

#define KEY0        HAL_GPIO_ReadPin(KEY0_GPIO_PORT, KEY0_GPIO_PIN)     /* Read KEY0 pin */
#define KEY1        HAL_GPIO_ReadPin(KEY1_GPIO_PORT, KEY1_GPIO_PIN)     /* Read KEY1 pin */
#define KEY2        HAL_GPIO_ReadPin(KEY2_GPIO_PORT, KEY2_GPIO_PIN)     /* Read KEY2 pin */
#define WK_UP       HAL_GPIO_ReadPin(WKUP_GPIO_PORT, WKUP_GPIO_PIN)     /* Read WKUP pin */

/* Key Press Status Definitions */
#define KEY0_PRES    1              /* KEY0 pressed */
#define KEY1_PRES    2              /* KEY1 pressed */
#define KEY2_PRES    3              /* KEY2 pressed */
#define WKUP_PRES    4              /* KEY_UP pressed (i.e., WK_UP) */

/******************************************************************************************/
/* External Interface Functions */

void key_init(void);                /* Key initialization function */
uint8_t key_scan(uint8_t mode);     /* Key scanning function */

#endif /* __KEY_H */
