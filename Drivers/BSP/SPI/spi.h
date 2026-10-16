#ifndef __SPI_H
#define __SPI_H
#include "./SYSTEM/sys/sys.h"

/******************************************************************************************/
/* SPI1 pin definitions */

#define SPI1_SCK_GPIO_PORT              GPIOB
#define SPI1_SCK_GPIO_PIN               GPIO_PIN_3
#define SPI1_SCK_GPIO_CLK_ENABLE()      do{ __HAL_RCC_GPIOB_CLK_ENABLE(); }while(0)   /* GPIO port clock enable */

#define SPI1_MISO_GPIO_PORT             GPIOB
#define SPI1_MISO_GPIO_PIN              GPIO_PIN_4
#define SPI1_MISO_GPIO_CLK_ENABLE()     do{ __HAL_RCC_GPIOB_CLK_ENABLE(); }while(0)   /* GPIO port clock enable */

#define SPI1_MOSI_GPIO_PORT             GPIOB
#define SPI1_MOSI_GPIO_PIN              GPIO_PIN_5
#define SPI1_MOSI_GPIO_CLK_ENABLE()     do{ __HAL_RCC_GPIOB_CLK_ENABLE(); }while(0)   /* GPIO port clock enable */

/* SPI1 related definitions */
#define SPI1_SPI                        SPI1
#define SPI1_SPI_CLK_ENABLE()           do{ __HAL_RCC_SPI1_CLK_ENABLE(); }while(0)    /* SPI1 clock enable */

/******************************************************************************************/
/* SPI bus speed settings */
#define SPI_SPEED_2         0           /* SPI clock = PCLK / 2 */
#define SPI_SPEED_4         1           /* SPI clock = PCLK / 4 */
#define SPI_SPEED_8         2           /* SPI clock = PCLK / 8 */
#define SPI_SPEED_16        3           /* SPI clock = PCLK / 16 */
#define SPI_SPEED_32        4           /* SPI clock = PCLK / 32 */
#define SPI_SPEED_64        5           /* SPI clock = PCLK / 64 */
#define SPI_SPEED_128       6           /* SPI clock = PCLK / 128 */
#define SPI_SPEED_256       7           /* SPI clock = PCLK / 256 */

/* Function declarations */
void spi1_init(void);                           /* Initialize SPI1 */
void spi1_set_speed(uint8_t speed);             /* Set SPI1 speed */
uint8_t spi1_read_write_byte(uint8_t txdata);   /* SPI1 read/write byte */

#endif
