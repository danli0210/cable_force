/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/** @addtogroup STM32F4xx_HAL_Driver
  * @{
  */

/** @defgroup I2CEx I2CEx
  * @brief I2C HAL module driver
  * @{
  */

#ifdef HAL_I2C_MODULE_ENABLED

#if  defined(I2C_FLTR_ANOFF)&&defined(I2C_FLTR_DNF)
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
/** @defgroup I2CEx_Exported_Functions I2C Exported Functions
  * @{
  */


/** @defgroup I2CEx_Exported_Functions_Group1 Extension features functions
 *  @brief   Extension features functions
 *
@verbatim
 ===============================================================================
                      ##### Extension features functions #####
 ===============================================================================
    [..] This section provides functions allowing to:
      (+) Configure Noise Filters

@endverbatim
  * @{
  */

/**
  * @brief  Configures I2C Analog noise filter.
  * @param  hi2c pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2Cx peripheral.
  * @param  AnalogFilter new state of the Analog filter.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_I2CEx_ConfigAnalogFilter(I2C_HandleTypeDef *hi2c, uint32_t AnalogFilter)
{
  /* Check the parameters */
  assert_param(IS_I2C_ALL_INSTANCE(hi2c->Instance));
  assert_param(IS_I2C_ANALOG_FILTER(AnalogFilter));

  if (hi2c->State == HAL_I2C_STATE_READY)
  {
    hi2c->State = HAL_I2C_STATE_BUSY;

    /* Disable the selected I2C peripheral */
    __HAL_I2C_DISABLE(hi2c);

    /* Reset I2Cx ANOFF bit */
    hi2c->Instance->FLTR &= ~(I2C_FLTR_ANOFF);

    /* Disable the analog filter */
    hi2c->Instance->FLTR |= AnalogFilter;

    __HAL_I2C_ENABLE(hi2c);

    hi2c->State = HAL_I2C_STATE_READY;

    return HAL_OK;
  }
  else
  {
    return HAL_BUSY;
  }
}

/**
  * @brief  Configures I2C Digital noise filter.
  * @param  hi2c pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2Cx peripheral.
  * @param  DigitalFilter Coefficient of digital noise filter between 0x00 and 0x0F.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_I2CEx_ConfigDigitalFilter(I2C_HandleTypeDef *hi2c, uint32_t DigitalFilter)
{
  uint16_t tmpreg = 0;

  /* Check the parameters */
  assert_param(IS_I2C_ALL_INSTANCE(hi2c->Instance));
  assert_param(IS_I2C_DIGITAL_FILTER(DigitalFilter));

  if (hi2c->State == HAL_I2C_STATE_READY)
  {
    hi2c->State = HAL_I2C_STATE_BUSY;

    /* Disable the selected I2C peripheral */
    __HAL_I2C_DISABLE(hi2c);

    /* Get the old register value */
    tmpreg = hi2c->Instance->FLTR;

    /* Reset I2Cx DNF bit [3:0] */
    tmpreg &= ~(I2C_FLTR_DNF);

    /* Set I2Cx DNF coefficient */
    tmpreg |= DigitalFilter;

    /* Store the new register value */
    hi2c->Instance->FLTR = tmpreg;

    __HAL_I2C_ENABLE(hi2c);

    hi2c->State = HAL_I2C_STATE_READY;

    return HAL_OK;
  }
  else
  {
    return HAL_BUSY;
  }
}

/**
  * @}
  */

/**
  * @}
  */
#endif

#endif /* HAL_I2C_MODULE_ENABLED */
/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
