/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/** @addtogroup STM32F4xx_HAL_Driver
  * @{
  */

/** @defgroup FLASH_RAMFUNC FLASH RAMFUNC
  * @brief FLASH functions executed from RAM
  * @{
  */
#ifdef HAL_FLASH_MODULE_ENABLED
#if defined(STM32F410Tx) || defined(STM32F410Cx) || defined(STM32F410Rx) || defined(STM32F411xE) || defined(STM32F446xx) || defined(STM32F412Zx) || defined(STM32F412Vx) || \
    defined(STM32F412Rx) || defined(STM32F412Cx)

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
/** @defgroup FLASH_RAMFUNC_Exported_Functions FLASH RAMFUNC Exported Functions
  * @{
  */

/** @defgroup FLASH_RAMFUNC_Exported_Functions_Group1 Peripheral features functions executed from internal RAM 
  *  @brief Peripheral Extended features functions 
  *
@verbatim   

 ===============================================================================
                      ##### ramfunc functions #####
 ===============================================================================  
    [..]
    This subsection provides a set of functions that should be executed from RAM 
    transfers.
    
@endverbatim
  * @{
  */

/**
  * @brief Stop the flash interface while System Run
  * @note  This mode is only available for STM32F41xxx/STM32F446xx devices. 
  * @note  This mode couldn't be set while executing with the flash itself. 
  *        It should be done with specific routine executed from RAM.     
  * @retval HAL status
  */
__RAM_FUNC HAL_StatusTypeDef HAL_FLASHEx_StopFlashInterfaceClk(void)
{
  /* Enable Power ctrl clock */
  __HAL_RCC_PWR_CLK_ENABLE();
  /* Stop the flash interface while System Run */  
  SET_BIT(PWR->CR, PWR_CR_FISSR);
   
  return HAL_OK;
}

/**
  * @brief Start the flash interface while System Run
  * @note  This mode is only available for STM32F411xx/STM32F446xx devices. 
  * @note  This mode couldn't be set while executing with the flash itself. 
  *        It should be done with specific routine executed from RAM.     
  * @retval HAL status
  */
__RAM_FUNC HAL_StatusTypeDef HAL_FLASHEx_StartFlashInterfaceClk(void)
{
  /* Enable Power ctrl clock */
  __HAL_RCC_PWR_CLK_ENABLE();
  /* Start the flash interface while System Run */
  CLEAR_BIT(PWR->CR, PWR_CR_FISSR);

  return HAL_OK;
}

/**
  * @brief Enable the flash sleep while System Run
  * @note  This mode is only available for STM32F41xxx/STM32F446xx devices. 
  * @note  This mode could n't be set while executing with the flash itself. 
  *        It should be done with specific routine executed from RAM.     
  * @retval HAL status
  */
__RAM_FUNC HAL_StatusTypeDef HAL_FLASHEx_EnableFlashSleepMode(void)
{
  /* Enable Power ctrl clock */
  __HAL_RCC_PWR_CLK_ENABLE();
  /* Enable the flash sleep while System Run */
  SET_BIT(PWR->CR, PWR_CR_FMSSR);

  return HAL_OK;
}

/**
  * @brief Disable the flash sleep while System Run
  * @note  This mode is only available for STM32F41xxx/STM32F446xx devices. 
  * @note  This mode couldn't be set while executing with the flash itself. 
  *        It should be done with specific routine executed from RAM.     
  * @retval HAL status
  */
__RAM_FUNC HAL_StatusTypeDef HAL_FLASHEx_DisableFlashSleepMode(void)
{
  /* Enable Power ctrl clock */
  __HAL_RCC_PWR_CLK_ENABLE();
  /* Disable the flash sleep while System Run */
  CLEAR_BIT(PWR->CR, PWR_CR_FMSSR);
  
  return HAL_OK;
}

/**
  * @}
  */

/**
  * @}
  */

#endif /* STM32F410xx || STM32F411xE || STM32F446xx || STM32F412Zx || STM32F412Vx || STM32F412Rx || STM32F412Cx */
#endif /* HAL_FLASH_MODULE_ENABLED */
/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
