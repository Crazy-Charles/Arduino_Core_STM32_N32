/**
  ******************************************************************************
  * @file    stm32f1xx_hal_rcc_ex.h
  * @author  MCD Application Team
  * @brief   Header file of RCC HAL Extension module.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2016 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32F1xx_HAL_RCC_EX_H
#define __STM32F1xx_HAL_RCC_EX_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal_def.h"

/** @addtogroup STM32F1xx_HAL_Driver
  * @{
  */

/** @addtogroup RCCEx
  * @{
  */

/** @addtogroup RCCEx_Private_Constants
 * @{
 */

#define CR_REG_INDEX                 ((uint8_t)1)

/**
  * @}
  */

/** @addtogroup RCCEx_Private_Macros
 * @{
 */

#define IS_RCC_HSE_PREDIV(__DIV__) (((__DIV__) == RCC_HSE_PREDIV_DIV1)  || ((__DIV__) == RCC_HSE_PREDIV_DIV2))

#define IS_RCC_PLL_MUL(MUL)                                                                                            \
    (((MUL) == RCC_PLL_MUL_2) || ((MUL) == RCC_PLL_MUL_3) || ((MUL) == RCC_PLL_MUL_4) || ((MUL) == RCC_PLL_MUL_5)      \
     || ((MUL) == RCC_PLL_MUL_6) || ((MUL) == RCC_PLL_MUL_7) || ((MUL) == RCC_PLL_MUL_8) || ((MUL) == RCC_PLL_MUL_9)   \
     || ((MUL) == RCC_PLL_MUL_10) || ((MUL) == RCC_PLL_MUL_11) || ((MUL) == RCC_PLL_MUL_12)                            \
     || ((MUL) == RCC_PLL_MUL_13) || ((MUL) == RCC_PLL_MUL_14) || ((MUL) == RCC_PLL_MUL_15)                            \
     || ((MUL) == RCC_PLL_MUL_16) || ((MUL) == RCC_PLL_MUL_17) || ((MUL) == RCC_PLL_MUL_18)                            \
     || ((MUL) == RCC_PLL_MUL_19) || ((MUL) == RCC_PLL_MUL_20) || ((MUL) == RCC_PLL_MUL_21)                            \
     || ((MUL) == RCC_PLL_MUL_22) || ((MUL) == RCC_PLL_MUL_23) || ((MUL) == RCC_PLL_MUL_24)                            \
     || ((MUL) == RCC_PLL_MUL_25) || ((MUL) == RCC_PLL_MUL_26) || ((MUL) == RCC_PLL_MUL_27)                            \
     || ((MUL) == RCC_PLL_MUL_28) || ((MUL) == RCC_PLL_MUL_29) || ((MUL) == RCC_PLL_MUL_30)                            \
     || ((MUL) == RCC_PLL_MUL_31) || ((MUL) == RCC_PLL_MUL_32))

#define IS_RCC_MCO1SOURCE(__SOURCE__) (((__SOURCE__) == RCC_MCO1SOURCE_SYSCLK)  || ((__SOURCE__) == RCC_MCO1SOURCE_HSI) \
                                    || ((__SOURCE__) == RCC_MCO1SOURCE_HSE)     || ((__SOURCE__) == RCC_MCO1SOURCE_PLLCLK) \
                                    || ((__SOURCE__) == RCC_MCO1SOURCE_NOCLOCK))


#define IS_RCC_ADCPLLCLK_DIV(__ADCCLK__) (((__ADCCLK__) == RCC_ADCPCLK2_DIV2)  || ((__ADCCLK__) == RCC_ADCPCLK2_DIV4)   || \
                                          ((__ADCCLK__) == RCC_ADCPCLK2_DIV6)  || ((__ADCCLK__) == RCC_ADCPCLK2_DIV8))

#define IS_RCC_I2S2CLKSOURCE(__SOURCE__) ((__SOURCE__) == RCC_I2S2CLKSOURCE_SYSCLK)

#define IS_RCC_I2S3CLKSOURCE(__SOURCE__) ((__SOURCE__) == RCC_I2S3CLKSOURCE_SYSCLK)

#define IS_RCC_PERIPHCLOCK(__SELECTION__)  \
               ((((__SELECTION__) & RCC_PERIPHCLK_RTC)  == RCC_PERIPHCLK_RTC)  || \
                (((__SELECTION__) & RCC_PERIPHCLK_ADC)  == RCC_PERIPHCLK_ADC)  || \
                (((__SELECTION__) & RCC_PERIPHCLK_I2S2) == RCC_PERIPHCLK_I2S2)  || \
                (((__SELECTION__) & RCC_PERIPHCLK_I2S3) == RCC_PERIPHCLK_I2S3)   || \
                (((__SELECTION__) & RCC_PERIPHCLK_USB)  == RCC_PERIPHCLK_USB))

#define IS_RCC_USBPLLCLK_DIV(__USBCLK__) (((__USBCLK__) == RCC_USBCLKSOURCE_PLL)  || ((__USBCLK__) == RCC_USBCLKSOURCE_PLL_DIV1_5))


/**
  * @}
  */

/* Exported types ------------------------------------------------------------*/

/** @defgroup RCCEx_Exported_Types RCCEx Exported Types
  * @{
  */

/**
  * @brief  RCC Internal/External Oscillator (HSE, HSI, LSE and LSI) configuration structure definition
  */
typedef struct
{
  uint32_t OscillatorType;       /*!< The oscillators to be configured.
                                       This parameter can be a value of @ref RCC_Oscillator_Type */

  uint32_t HSEState;              /*!< The new state of the HSE.
                                       This parameter can be a value of @ref RCC_HSE_Config */

  uint32_t HSEPredivValue;       /*!<  The Prediv1 factor value (named PREDIV1 or PLLXTPRE in RM)
                                       This parameter can be a value of @ref RCCEx_Prediv1_Factor */

  uint32_t LSEState;              /*!<  The new state of the LSE.
                                        This parameter can be a value of @ref RCC_LSE_Config */

  uint32_t HSIState;              /*!< The new state of the HSI.
                                       This parameter can be a value of @ref RCC_HSI_Config */

  uint32_t HSICalibrationValue;   /*!< The HSI calibration trimming value (default is RCC_HSICALIBRATION_DEFAULT).
                                       This parameter must be a number between Min_Data = 0x00 and Max_Data = 0x1F */

  uint32_t LSIState;              /*!<  The new state of the LSI.
                                        This parameter can be a value of @ref RCC_LSI_Config */

  RCC_PLLInitTypeDef PLL;         /*!< PLL structure parameters */

} RCC_OscInitTypeDef;

/**
  * @brief  RCC extended clocks structure definition
  */
typedef struct
{
  uint32_t PeriphClockSelection;      /*!< The Extended Clock to be configured.
                                       This parameter can be a value of @ref RCCEx_Periph_Clock_Selection */

  uint32_t RTCClockSelection;         /*!< specifies the RTC clock source.
                                       This parameter can be a value of @ref RCC_RTC_Clock_Source */

  uint32_t AdcClockSelection;         /*!< ADC clock source
                                       This parameter can be a value of @ref RCCEx_ADC_Prescaler */

  uint32_t I2s2ClockSelection;         /*!< I2S2 clock source
                                       This parameter can be a value of @ref RCCEx_I2S2_Clock_Source */

  uint32_t I2s3ClockSelection;         /*!< I2S3 clock source
                                       This parameter can be a value of @ref RCCEx_I2S3_Clock_Source */

  uint32_t UsbClockSelection;         /*!< USB clock source
                                       This parameter can be a value of @ref RCCEx_USB_Prescaler */
} RCC_PeriphCLKInitTypeDef;

/**
  * @}
  */

/* Exported constants --------------------------------------------------------*/

/** @defgroup RCCEx_Exported_Constants RCCEx Exported Constants
  * @{
  */

/** @defgroup RCCEx_Periph_Clock_Selection Periph Clock Selection
  * @{
  */
#define RCC_PERIPHCLK_RTC           0x00000001U
#define RCC_PERIPHCLK_ADC           0x00000002U

#define RCC_PERIPHCLK_I2S2          0x00000004U
#define RCC_PERIPHCLK_I2S3          0x00000008U

#define RCC_PERIPHCLK_USB           0x00000010U

/** @defgroup RCCEx_ADC_Prescaler ADC Prescaler
  * @{
  */
#define RCC_ADCPCLK2_DIV2              RCC_CFGR_ADCPRE_DIV2
#define RCC_ADCPCLK2_DIV4              RCC_CFGR_ADCPRE_DIV4
#define RCC_ADCPCLK2_DIV6              RCC_CFGR_ADCPRE_DIV6
#define RCC_ADCPCLK2_DIV8              RCC_CFGR_ADCPRE_DIV8

/** @defgroup RCCEx_I2S2_Clock_Source I2S2 Clock Source
  * @{
  */
#define RCC_I2S2CLKSOURCE_SYSCLK              0x00000000U

/** @defgroup RCCEx_I2S3_Clock_Source I2S3 Clock Source
  * @{
  */
#define RCC_I2S3CLKSOURCE_SYSCLK              0x00000000U

/** @defgroup RCCEx_USB_Prescaler USB Prescaler
  * @{
  */
#define RCC_USBCLKSOURCE_PLL              RCC_CFGR_USBPRE
#define RCC_USBCLKSOURCE_PLL_DIV1_5       0x00000000U

/** @defgroup RCCEx_Prediv1_Factor HSE Prediv1 Factor
  * @{
  */

#define RCC_HSE_PREDIV_DIV1              0x00000000U
#define RCC_HSE_PREDIV_DIV2              RCC_CFGR_PLLXTPRE

/** @defgroup RCCEx_PLL_Multiplication_Factor PLL Multiplication Factor
  * @{
  */

#define RCC_PLL_MUL2                    RCC_CFGR_PLLMULL2
#define RCC_PLL_MUL3                    RCC_CFGR_PLLMULL3
#define RCC_PLL_MUL4                    RCC_CFGR_PLLMULL4
#define RCC_PLL_MUL5                    RCC_CFGR_PLLMULL5
#define RCC_PLL_MUL6                    RCC_CFGR_PLLMULL6
#define RCC_PLL_MUL7                    RCC_CFGR_PLLMULL7
#define RCC_PLL_MUL8                    RCC_CFGR_PLLMULL8
#define RCC_PLL_MUL9                    RCC_CFGR_PLLMULL9
#define RCC_PLL_MUL10                   RCC_CFGR_PLLMULL10
#define RCC_PLL_MUL11                   RCC_CFGR_PLLMULL11
#define RCC_PLL_MUL12                   RCC_CFGR_PLLMULL12
#define RCC_PLL_MUL13                   RCC_CFGR_PLLMULL13
#define RCC_PLL_MUL14                   RCC_CFGR_PLLMULL14
#define RCC_PLL_MUL15                   RCC_CFGR_PLLMULL15
#define RCC_PLL_MUL16                   RCC_CFGR_PLLMULL16
#define RCC_PLL_MUL17 ((uint32_t)0x08000000)
#define RCC_PLL_MUL18 ((uint32_t)0x08040000)
#define RCC_PLL_MUL19 ((uint32_t)0x08080000)
#define RCC_PLL_MUL20 ((uint32_t)0x080C0000)
#define RCC_PLL_MUL21 ((uint32_t)0x08100000)
#define RCC_PLL_MUL22 ((uint32_t)0x08140000)
#define RCC_PLL_MUL23 ((uint32_t)0x08180000)
#define RCC_PLL_MUL24 ((uint32_t)0x081C0000)
#define RCC_PLL_MUL25 ((uint32_t)0x08200000)
#define RCC_PLL_MUL26 ((uint32_t)0x08240000)
#define RCC_PLL_MUL27 ((uint32_t)0x08280000)
#define RCC_PLL_MUL28 ((uint32_t)0x082C0000)
#define RCC_PLL_MUL29 ((uint32_t)0x08300000)
#define RCC_PLL_MUL30 ((uint32_t)0x08340000)
#define RCC_PLL_MUL31 ((uint32_t)0x08380000)
#define RCC_PLL_MUL32 ((uint32_t)0x083C0000)

/**
  * @}
  */

/** @defgroup RCCEx_MCO1_Clock_Source MCO1 Clock Source
  * @{
  */
#define RCC_MCO1SOURCE_NOCLOCK           ((uint32_t)RCC_CFGR_MCO_NOCLOCK)
#define RCC_MCO1SOURCE_SYSCLK            ((uint32_t)RCC_CFGR_MCO_SYSCLK)
#define RCC_MCO1SOURCE_HSI               ((uint32_t)RCC_CFGR_MCO_HSI)
#define RCC_MCO1SOURCE_HSE               ((uint32_t)RCC_CFGR_MCO_HSE)
#define RCC_MCO1SOURCE_PLLCLK            ((uint32_t)RCC_CFGR_MCO_PLLCLK_DIV2)

/* Exported macro ------------------------------------------------------------*/
/** @defgroup RCCEx_Exported_Macros RCCEx Exported Macros
 * @{
 */

/** @defgroup RCCEx_Peripheral_Clock_Enable_Disable Peripheral Clock Enable Disable
  * @brief  Enable or disable the AHB1 peripheral clock.
  * @note   After reset, the peripheral clock (used for registers read/write access)
  *         is disabled and the application software has to enable this clock before
  *         using it.
  * @{
  */

#define __HAL_RCC_DMA2_CLK_ENABLE()   do { \
                                        __IO uint32_t tmpreg; \
                                        SET_BIT(RCC->AHBENR, RCC_AHBENR_DMA2EN);\
                                        /* Delay after an RCC peripheral clock enabling */ \
                                        tmpreg = READ_BIT(RCC->AHBENR, RCC_AHBENR_DMA2EN);\
                                        UNUSED(tmpreg); \
                                      } while(0U)

#define __HAL_RCC_DMA2_CLK_DISABLE()        (RCC->AHBENR &= ~(RCC_AHBENR_DMA2EN))



#define __HAL_RCC_FSMC_CLK_ENABLE()   do { \
                                        __IO uint32_t tmpreg; \
                                        SET_BIT(RCC->AHBENR, RCC_AHBENR_FSMCEN);\
                                        /* Delay after an RCC peripheral clock enabling */ \
                                        tmpreg = READ_BIT(RCC->AHBENR, RCC_AHBENR_FSMCEN);\
                                        UNUSED(tmpreg); \
                                      } while(0U)

#define __HAL_RCC_FSMC_CLK_DISABLE()        (RCC->AHBENR &= ~(RCC_AHBENR_FSMCEN))



#define __HAL_RCC_SDIO_CLK_ENABLE()   do { \
                                        __IO uint32_t tmpreg; \
                                        SET_BIT(RCC->AHBENR, RCC_AHBENR_SDIOEN);\
                                        /* Delay after an RCC peripheral clock enabling */ \
                                        tmpreg = READ_BIT(RCC->AHBENR, RCC_AHBENR_SDIOEN);\
                                        UNUSED(tmpreg); \
                                      } while(0U)


#define __HAL_RCC_SDIO_CLK_DISABLE()        (RCC->AHBENR &= ~(RCC_AHBENR_SDIOEN))



/** @defgroup RCCEx_AHB1_Peripheral_Clock_Enable_Disable_Status AHB1 Peripheral Clock Enable Disable Status
  * @brief  Get the enable or disable status of the AHB1 peripheral clock.
  * @note   After reset, the peripheral clock (used for registers read/write access)
  *         is disabled and the application software has to enable this clock before
  *         using it.
  * @{
  */


#define __HAL_RCC_DMA2_IS_CLK_ENABLED()       ((RCC->AHBENR & (RCC_AHBENR_DMA2EN)) != RESET)
#define __HAL_RCC_DMA2_IS_CLK_DISABLED()      ((RCC->AHBENR & (RCC_AHBENR_DMA2EN)) == RESET)

#define __HAL_RCC_FSMC_IS_CLK_ENABLED()       ((RCC->AHBENR & (RCC_AHBENR_FSMCEN)) != RESET)
#define __HAL_RCC_FSMC_IS_CLK_DISABLED()      ((RCC->AHBENR & (RCC_AHBENR_FSMCEN)) == RESET)

#define __HAL_RCC_SDIO_IS_CLK_ENABLED()       ((RCC->AHBENR & (RCC_AHBENR_SDIOEN)) != RESET)
#define __HAL_RCC_SDIO_IS_CLK_DISABLED()      ((RCC->AHBENR & (RCC_AHBENR_SDIOEN)) == RESET)




/**
  * @}
  */

/** @defgroup RCCEx_APB1_Clock_Enable_Disable APB1 Clock Enable Disable
  * @brief  Enable or disable the Low Speed APB (APB1) peripheral clock.
  * @note   After reset, the peripheral clock (used for registers read/write access)
  *         is disabled and the application software has to enable this clock before
  *         using it.
  * @{
  */


#define __HAL_RCC_CAN1_CLK_ENABLE()   do { \
                                        __IO uint32_t tmpreg; \
                                        SET_BIT(RCC->APB1ENR, RCC_APB1ENR_CAN1EN);\
                                        /* Delay after an RCC peripheral clock enabling */ \
                                        tmpreg = READ_BIT(RCC->APB1ENR, RCC_APB1ENR_CAN1EN);\
                                        UNUSED(tmpreg); \
                                      } while(0U)

#define __HAL_RCC_CAN1_CLK_DISABLE()        (RCC->APB1ENR &= ~(RCC_APB1ENR_CAN1EN))



#define __HAL_RCC_TIM4_CLK_ENABLE()   do { \
                                        __IO uint32_t tmpreg; \
                                        SET_BIT(RCC->APB1ENR, RCC_APB1ENR_TIM4EN);\
                                        /* Delay after an RCC peripheral clock enabling */ \
                                        tmpreg = READ_BIT(RCC->APB1ENR, RCC_APB1ENR_TIM4EN);\
                                        UNUSED(tmpreg); \
                                      } while(0U)

#define __HAL_RCC_SPI2_CLK_ENABLE()   do { \
                                        __IO uint32_t tmpreg; \
                                        SET_BIT(RCC->APB1ENR, RCC_APB1ENR_SPI2EN);\
                                        /* Delay after an RCC peripheral clock enabling */ \
                                        tmpreg = READ_BIT(RCC->APB1ENR, RCC_APB1ENR_SPI2EN);\
                                        UNUSED(tmpreg); \
                                      } while(0U)

#define __HAL_RCC_USART3_CLK_ENABLE()   do { \
                                        __IO uint32_t tmpreg; \
                                        SET_BIT(RCC->APB1ENR, RCC_APB1ENR_USART3EN);\
                                        /* Delay after an RCC peripheral clock enabling */ \
                                        tmpreg = READ_BIT(RCC->APB1ENR, RCC_APB1ENR_USART3EN);\
                                        UNUSED(tmpreg); \
                                      } while(0U)

#define __HAL_RCC_I2C2_CLK_ENABLE()   do { \
                                        __IO uint32_t tmpreg; \
                                        SET_BIT(RCC->APB1ENR, RCC_APB1ENR_I2C2EN);\
                                        /* Delay after an RCC peripheral clock enabling */ \
                                        tmpreg = READ_BIT(RCC->APB1ENR, RCC_APB1ENR_I2C2EN);\
                                        UNUSED(tmpreg); \
                                      } while(0U)

#define __HAL_RCC_TIM4_CLK_DISABLE()        (RCC->APB1ENR &= ~(RCC_APB1ENR_TIM4EN))
#define __HAL_RCC_SPI2_CLK_DISABLE()        (RCC->APB1ENR &= ~(RCC_APB1ENR_SPI2EN))
#define __HAL_RCC_USART3_CLK_DISABLE()      (RCC->APB1ENR &= ~(RCC_APB1ENR_USART3EN))
#define __HAL_RCC_I2C2_CLK_DISABLE()        (RCC->APB1ENR &= ~(RCC_APB1ENR_I2C2EN))



#define __HAL_RCC_USB_CLK_ENABLE()   do { \
                                        __IO uint32_t tmpreg; \
                                        SET_BIT(RCC->APB1ENR, RCC_APB1ENR_USBEN);\
                                        /* Delay after an RCC peripheral clock enabling */ \
                                        tmpreg = READ_BIT(RCC->APB1ENR, RCC_APB1ENR_USBEN);\
                                        UNUSED(tmpreg); \
                                      } while(0U)

#define __HAL_RCC_USB_CLK_DISABLE()         (RCC->APB1ENR &= ~(RCC_APB1ENR_USBEN))


#define __HAL_RCC_TIM5_CLK_ENABLE()   do { \
                                        __IO uint32_t tmpreg; \
                                        SET_BIT(RCC->APB1ENR, RCC_APB1ENR_TIM5EN);\
                                        /* Delay after an RCC peripheral clock enabling */ \
                                        tmpreg = READ_BIT(RCC->APB1ENR, RCC_APB1ENR_TIM5EN);\
                                        UNUSED(tmpreg); \
                                      } while(0U)

#define __HAL_RCC_TIM6_CLK_ENABLE()   do { \
                                        __IO uint32_t tmpreg; \
                                        SET_BIT(RCC->APB1ENR, RCC_APB1ENR_TIM6EN);\
                                        /* Delay after an RCC peripheral clock enabling */ \
                                        tmpreg = READ_BIT(RCC->APB1ENR, RCC_APB1ENR_TIM6EN);\
                                        UNUSED(tmpreg); \
                                      } while(0U)

#define __HAL_RCC_TIM7_CLK_ENABLE()   do { \
                                        __IO uint32_t tmpreg; \
                                        SET_BIT(RCC->APB1ENR, RCC_APB1ENR_TIM7EN);\
                                        /* Delay after an RCC peripheral clock enabling */ \
                                        tmpreg = READ_BIT(RCC->APB1ENR, RCC_APB1ENR_TIM7EN);\
                                        UNUSED(tmpreg); \
                                      } while(0U)

#define __HAL_RCC_SPI3_CLK_ENABLE()   do { \
                                        __IO uint32_t tmpreg; \
                                        SET_BIT(RCC->APB1ENR, RCC_APB1ENR_SPI3EN);\
                                        /* Delay after an RCC peripheral clock enabling */ \
                                        tmpreg = READ_BIT(RCC->APB1ENR, RCC_APB1ENR_SPI3EN);\
                                        UNUSED(tmpreg); \
                                      } while(0U)

#define __HAL_RCC_UART4_CLK_ENABLE()   do { \
                                        __IO uint32_t tmpreg; \
                                        SET_BIT(RCC->APB1ENR, RCC_APB1ENR_UART4EN);\
                                        /* Delay after an RCC peripheral clock enabling */ \
                                        tmpreg = READ_BIT(RCC->APB1ENR, RCC_APB1ENR_UART4EN);\
                                        UNUSED(tmpreg); \
                                      } while(0U)

#define __HAL_RCC_UART5_CLK_ENABLE()   do { \
                                        __IO uint32_t tmpreg; \
                                        SET_BIT(RCC->APB1ENR, RCC_APB1ENR_UART5EN);\
                                        /* Delay after an RCC peripheral clock enabling */ \
                                        tmpreg = READ_BIT(RCC->APB1ENR, RCC_APB1ENR_UART5EN);\
                                        UNUSED(tmpreg); \
                                      } while(0U)

#define __HAL_RCC_DAC_CLK_ENABLE()   do { \
                                        __IO uint32_t tmpreg; \
                                        SET_BIT(RCC->APB1ENR, RCC_APB1ENR_DACEN);\
                                        /* Delay after an RCC peripheral clock enabling */ \
                                        tmpreg = READ_BIT(RCC->APB1ENR, RCC_APB1ENR_DACEN);\
                                        UNUSED(tmpreg); \
                                      } while(0U)

#define __HAL_RCC_TIM5_CLK_DISABLE()        (RCC->APB1ENR &= ~(RCC_APB1ENR_TIM5EN))
#define __HAL_RCC_TIM6_CLK_DISABLE()        (RCC->APB1ENR &= ~(RCC_APB1ENR_TIM6EN))
#define __HAL_RCC_TIM7_CLK_DISABLE()        (RCC->APB1ENR &= ~(RCC_APB1ENR_TIM7EN))
#define __HAL_RCC_SPI3_CLK_DISABLE()        (RCC->APB1ENR &= ~(RCC_APB1ENR_SPI3EN))
#define __HAL_RCC_UART4_CLK_DISABLE()       (RCC->APB1ENR &= ~(RCC_APB1ENR_UART4EN))
#define __HAL_RCC_UART5_CLK_DISABLE()       (RCC->APB1ENR &= ~(RCC_APB1ENR_UART5EN))
#define __HAL_RCC_DAC_CLK_DISABLE()         (RCC->APB1ENR &= ~(RCC_APB1ENR_DACEN))


/** @defgroup RCCEx_APB1_Peripheral_Clock_Enable_Disable_Status APB1 Peripheral Clock Enable Disable Status
  * @brief  Get the enable or disable status of the APB1 peripheral clock.
  * @note   After reset, the peripheral clock (used for registers read/write access)
  *         is disabled and the application software has to enable this clock before
  *         using it.
  * @{
  */

#define __HAL_RCC_CAN1_IS_CLK_ENABLED()       ((RCC->APB1ENR & (RCC_APB1ENR_CAN1EN)) != RESET)
#define __HAL_RCC_CAN1_IS_CLK_DISABLED()      ((RCC->APB1ENR & (RCC_APB1ENR_CAN1EN)) == RESET)

#define __HAL_RCC_TIM4_IS_CLK_ENABLED()       ((RCC->APB1ENR & (RCC_APB1ENR_TIM4EN)) != RESET)
#define __HAL_RCC_TIM4_IS_CLK_DISABLED()      ((RCC->APB1ENR & (RCC_APB1ENR_TIM4EN)) == RESET)
#define __HAL_RCC_SPI2_IS_CLK_ENABLED()       ((RCC->APB1ENR & (RCC_APB1ENR_SPI2EN)) != RESET)
#define __HAL_RCC_SPI2_IS_CLK_DISABLED()      ((RCC->APB1ENR & (RCC_APB1ENR_SPI2EN)) == RESET)
#define __HAL_RCC_USART3_IS_CLK_ENABLED()       ((RCC->APB1ENR & (RCC_APB1ENR_USART3EN)) != RESET)
#define __HAL_RCC_USART3_IS_CLK_DISABLED()      ((RCC->APB1ENR & (RCC_APB1ENR_USART3EN)) == RESET)
#define __HAL_RCC_I2C2_IS_CLK_ENABLED()       ((RCC->APB1ENR & (RCC_APB1ENR_I2C2EN)) != RESET)
#define __HAL_RCC_I2C2_IS_CLK_DISABLED()      ((RCC->APB1ENR & (RCC_APB1ENR_I2C2EN)) == RESET)

#define __HAL_RCC_USB_IS_CLK_ENABLED()       ((RCC->APB1ENR & (RCC_APB1ENR_USBEN)) != RESET)
#define __HAL_RCC_USB_IS_CLK_DISABLED()      ((RCC->APB1ENR & (RCC_APB1ENR_USBEN)) == RESET)

#define __HAL_RCC_TIM5_IS_CLK_ENABLED()       ((RCC->APB1ENR & (RCC_APB1ENR_TIM5EN)) != RESET)
#define __HAL_RCC_TIM5_IS_CLK_DISABLED()      ((RCC->APB1ENR & (RCC_APB1ENR_TIM5EN)) == RESET)
#define __HAL_RCC_TIM6_IS_CLK_ENABLED()       ((RCC->APB1ENR & (RCC_APB1ENR_TIM6EN)) != RESET)
#define __HAL_RCC_TIM6_IS_CLK_DISABLED()      ((RCC->APB1ENR & (RCC_APB1ENR_TIM6EN)) == RESET)
#define __HAL_RCC_TIM7_IS_CLK_ENABLED()       ((RCC->APB1ENR & (RCC_APB1ENR_TIM7EN)) != RESET)
#define __HAL_RCC_TIM7_IS_CLK_DISABLED()      ((RCC->APB1ENR & (RCC_APB1ENR_TIM7EN)) == RESET)
#define __HAL_RCC_SPI3_IS_CLK_ENABLED()       ((RCC->APB1ENR & (RCC_APB1ENR_SPI3EN)) != RESET)
#define __HAL_RCC_SPI3_IS_CLK_DISABLED()      ((RCC->APB1ENR & (RCC_APB1ENR_SPI3EN)) == RESET)
#define __HAL_RCC_UART4_IS_CLK_ENABLED()       ((RCC->APB1ENR & (RCC_APB1ENR_UART4EN)) != RESET)
#define __HAL_RCC_UART4_IS_CLK_DISABLED()      ((RCC->APB1ENR & (RCC_APB1ENR_UART4EN)) == RESET)
#define __HAL_RCC_UART5_IS_CLK_ENABLED()       ((RCC->APB1ENR & (RCC_APB1ENR_UART5EN)) != RESET)
#define __HAL_RCC_UART5_IS_CLK_DISABLED()      ((RCC->APB1ENR & (RCC_APB1ENR_UART5EN)) == RESET)
#define __HAL_RCC_DAC_IS_CLK_ENABLED()       ((RCC->APB1ENR & (RCC_APB1ENR_DACEN)) != RESET)
#define __HAL_RCC_DAC_IS_CLK_DISABLED()      ((RCC->APB1ENR & (RCC_APB1ENR_DACEN)) == RESET)



/**
  * @}
  */

/** @defgroup RCCEx_APB2_Clock_Enable_Disable APB2 Clock Enable Disable
  * @brief  Enable or disable the High Speed APB (APB2) peripheral clock.
  * @note   After reset, the peripheral clock (used for registers read/write access)
  *         is disabled and the application software has to enable this clock before
  *         using it.
  * @{
  */

#define __HAL_RCC_ADC2_CLK_ENABLE()   do { \
                                        __IO uint32_t tmpreg; \
                                        SET_BIT(RCC->APB2ENR, RCC_APB2ENR_ADC2EN);\
                                        /* Delay after an RCC peripheral clock enabling */ \
                                        tmpreg = READ_BIT(RCC->APB2ENR, RCC_APB2ENR_ADC2EN);\
                                        UNUSED(tmpreg); \
                                      } while(0U)

#define __HAL_RCC_ADC2_CLK_DISABLE()        (RCC->APB2ENR &= ~(RCC_APB2ENR_ADC2EN))


#define __HAL_RCC_GPIOE_CLK_ENABLE()   do { \
                                        __IO uint32_t tmpreg; \
                                        SET_BIT(RCC->APB2ENR, RCC_APB2ENR_IOPEEN);\
                                        /* Delay after an RCC peripheral clock enabling */ \
                                        tmpreg = READ_BIT(RCC->APB2ENR, RCC_APB2ENR_IOPEEN);\
                                        UNUSED(tmpreg); \
                                      } while(0U)

#define __HAL_RCC_GPIOE_CLK_DISABLE()       (RCC->APB2ENR &= ~(RCC_APB2ENR_IOPEEN))

#define __HAL_RCC_GPIOF_CLK_ENABLE()   do { \
                                        __IO uint32_t tmpreg; \
                                        SET_BIT(RCC->APB2ENR, RCC_APB2ENR_IOPFEN);\
                                        /* Delay after an RCC peripheral clock enabling */ \
                                        tmpreg = READ_BIT(RCC->APB2ENR, RCC_APB2ENR_IOPFEN);\
                                        UNUSED(tmpreg); \
                                      } while(0U)

#define __HAL_RCC_GPIOG_CLK_ENABLE()   do { \
                                        __IO uint32_t tmpreg; \
                                        SET_BIT(RCC->APB2ENR, RCC_APB2ENR_IOPGEN);\
                                        /* Delay after an RCC peripheral clock enabling */ \
                                        tmpreg = READ_BIT(RCC->APB2ENR, RCC_APB2ENR_IOPGEN);\
                                        UNUSED(tmpreg); \
                                      } while(0U)

#define __HAL_RCC_GPIOF_CLK_DISABLE()       (RCC->APB2ENR &= ~(RCC_APB2ENR_IOPFEN))
#define __HAL_RCC_GPIOG_CLK_DISABLE()       (RCC->APB2ENR &= ~(RCC_APB2ENR_IOPGEN))

#define __HAL_RCC_TIM8_CLK_ENABLE()   do { \
                                        __IO uint32_t tmpreg; \
                                        SET_BIT(RCC->APB2ENR, RCC_APB2ENR_TIM8EN);\
                                        /* Delay after an RCC peripheral clock enabling */ \
                                        tmpreg = READ_BIT(RCC->APB2ENR, RCC_APB2ENR_TIM8EN);\
                                        UNUSED(tmpreg); \
                                      } while(0U)

#define __HAL_RCC_ADC3_CLK_ENABLE()   do { \
                                        __IO uint32_t tmpreg; \
                                        SET_BIT(RCC->APB2ENR, RCC_APB2ENR_ADC3EN);\
                                        /* Delay after an RCC peripheral clock enabling */ \
                                        tmpreg = READ_BIT(RCC->APB2ENR, RCC_APB2ENR_ADC3EN);\
                                        UNUSED(tmpreg); \
                                      } while(0U)

#define __HAL_RCC_TIM8_CLK_DISABLE()        (RCC->APB2ENR &= ~(RCC_APB2ENR_TIM8EN))
#define __HAL_RCC_ADC3_CLK_DISABLE()        (RCC->APB2ENR &= ~(RCC_APB2ENR_ADC3EN))




/**
  * @}
  */

/** @defgroup RCCEx_APB2_Peripheral_Clock_Enable_Disable_Status APB2 Peripheral Clock Enable Disable Status
  * @brief  Get the enable or disable status of the APB2 peripheral clock.
  * @note   After reset, the peripheral clock (used for registers read/write access)
  *         is disabled and the application software has to enable this clock before
  *         using it.
  * @{
  */

#define __HAL_RCC_ADC2_IS_CLK_ENABLED()       ((RCC->APB2ENR & (RCC_APB2ENR_ADC2EN)) != RESET)
#define __HAL_RCC_ADC2_IS_CLK_DISABLED()      ((RCC->APB2ENR & (RCC_APB2ENR_ADC2EN)) == RESET)

#define __HAL_RCC_GPIOE_IS_CLK_ENABLED()       ((RCC->APB2ENR & (RCC_APB2ENR_IOPEEN)) != RESET)
#define __HAL_RCC_GPIOE_IS_CLK_DISABLED()      ((RCC->APB2ENR & (RCC_APB2ENR_IOPEEN)) == RESET)

#define __HAL_RCC_GPIOF_IS_CLK_ENABLED()       ((RCC->APB2ENR & (RCC_APB2ENR_IOPFEN)) != RESET)
#define __HAL_RCC_GPIOF_IS_CLK_DISABLED()      ((RCC->APB2ENR & (RCC_APB2ENR_IOPFEN)) == RESET)

#define __HAL_RCC_GPIOG_IS_CLK_ENABLED()       ((RCC->APB2ENR & (RCC_APB2ENR_IOPGEN)) != RESET)
#define __HAL_RCC_GPIOG_IS_CLK_DISABLED()      ((RCC->APB2ENR & (RCC_APB2ENR_IOPGEN)) == RESET)

#define __HAL_RCC_TIM8_IS_CLK_ENABLED()       ((RCC->APB2ENR & (RCC_APB2ENR_TIM8EN)) != RESET)
#define __HAL_RCC_TIM8_IS_CLK_DISABLED()      ((RCC->APB2ENR & (RCC_APB2ENR_TIM8EN)) == RESET)

#define __HAL_RCC_ADC3_IS_CLK_ENABLED()       ((RCC->APB2ENR & (RCC_APB2ENR_ADC3EN)) != RESET)
#define __HAL_RCC_ADC3_IS_CLK_DISABLED()      ((RCC->APB2ENR & (RCC_APB2ENR_ADC3EN)) == RESET)


/** @defgroup RCCEx_APB1_Force_Release_Reset APB1 Force Release Reset
  * @brief  Force or release APB1 peripheral reset.
  * @{
  */

#define __HAL_RCC_CAN1_FORCE_RESET()        (RCC->APB1RSTR |= (RCC_APB1RSTR_CAN1RST))
#define __HAL_RCC_CAN1_RELEASE_RESET()      (RCC->APB1RSTR &= ~(RCC_APB1RSTR_CAN1RST))

#define __HAL_RCC_TIM4_FORCE_RESET()        (RCC->APB1RSTR |= (RCC_APB1RSTR_TIM4RST))
#define __HAL_RCC_SPI2_FORCE_RESET()        (RCC->APB1RSTR |= (RCC_APB1RSTR_SPI2RST))
#define __HAL_RCC_USART3_FORCE_RESET()      (RCC->APB1RSTR |= (RCC_APB1RSTR_USART3RST))
#define __HAL_RCC_I2C2_FORCE_RESET()        (RCC->APB1RSTR |= (RCC_APB1RSTR_I2C2RST))

#define __HAL_RCC_TIM4_RELEASE_RESET()      (RCC->APB1RSTR &= ~(RCC_APB1RSTR_TIM4RST))
#define __HAL_RCC_SPI2_RELEASE_RESET()      (RCC->APB1RSTR &= ~(RCC_APB1RSTR_SPI2RST))
#define __HAL_RCC_USART3_RELEASE_RESET()    (RCC->APB1RSTR &= ~(RCC_APB1RSTR_USART3RST))
#define __HAL_RCC_I2C2_RELEASE_RESET()      (RCC->APB1RSTR &= ~(RCC_APB1RSTR_I2C2RST))

#define __HAL_RCC_USB_FORCE_RESET()         (RCC->APB1RSTR |= (RCC_APB1RSTR_USBRST))
#define __HAL_RCC_USB_RELEASE_RESET()       (RCC->APB1RSTR &= ~(RCC_APB1RSTR_USBRST))


#define __HAL_RCC_TIM5_FORCE_RESET()        (RCC->APB1RSTR |= (RCC_APB1RSTR_TIM5RST))
#define __HAL_RCC_TIM6_FORCE_RESET()        (RCC->APB1RSTR |= (RCC_APB1RSTR_TIM6RST))
#define __HAL_RCC_TIM7_FORCE_RESET()        (RCC->APB1RSTR |= (RCC_APB1RSTR_TIM7RST))
#define __HAL_RCC_SPI3_FORCE_RESET()        (RCC->APB1RSTR |= (RCC_APB1RSTR_SPI3RST))
#define __HAL_RCC_UART4_FORCE_RESET()       (RCC->APB1RSTR |= (RCC_APB1RSTR_UART4RST))
#define __HAL_RCC_UART5_FORCE_RESET()       (RCC->APB1RSTR |= (RCC_APB1RSTR_UART5RST))
#define __HAL_RCC_DAC_FORCE_RESET()         (RCC->APB1RSTR |= (RCC_APB1RSTR_DACRST))

#define __HAL_RCC_TIM5_RELEASE_RESET()      (RCC->APB1RSTR &= ~(RCC_APB1RSTR_TIM5RST))
#define __HAL_RCC_TIM6_RELEASE_RESET()      (RCC->APB1RSTR &= ~(RCC_APB1RSTR_TIM6RST))
#define __HAL_RCC_TIM7_RELEASE_RESET()      (RCC->APB1RSTR &= ~(RCC_APB1RSTR_TIM7RST))
#define __HAL_RCC_SPI3_RELEASE_RESET()      (RCC->APB1RSTR &= ~(RCC_APB1RSTR_SPI3RST))
#define __HAL_RCC_UART4_RELEASE_RESET()     (RCC->APB1RSTR &= ~(RCC_APB1RSTR_UART4RST))
#define __HAL_RCC_UART5_RELEASE_RESET()     (RCC->APB1RSTR &= ~(RCC_APB1RSTR_UART5RST))
#define __HAL_RCC_DAC_RELEASE_RESET()       (RCC->APB1RSTR &= ~(RCC_APB1RSTR_DACRST))



/**
  * @}
  */

/** @defgroup RCCEx_APB2_Force_Release_Reset APB2 Force Release Reset
  * @brief  Force or release APB2 peripheral reset.
  * @{
  */

#define __HAL_RCC_ADC2_FORCE_RESET()        (RCC->APB2RSTR |= (RCC_APB2RSTR_ADC2RST))
#define __HAL_RCC_ADC2_RELEASE_RESET()      (RCC->APB2RSTR &= ~(RCC_APB2RSTR_ADC2RST))

#define __HAL_RCC_GPIOE_FORCE_RESET()       (RCC->APB2RSTR |= (RCC_APB2RSTR_IOPERST))
#define __HAL_RCC_GPIOE_RELEASE_RESET()     (RCC->APB2RSTR &= ~(RCC_APB2RSTR_IOPERST))

#define __HAL_RCC_GPIOF_FORCE_RESET()       (RCC->APB2RSTR |= (RCC_APB2RSTR_IOPFRST))
#define __HAL_RCC_GPIOG_FORCE_RESET()       (RCC->APB2RSTR |= (RCC_APB2RSTR_IOPGRST))

#define __HAL_RCC_GPIOF_RELEASE_RESET()     (RCC->APB2RSTR &= ~(RCC_APB2RSTR_IOPFRST))
#define __HAL_RCC_GPIOG_RELEASE_RESET()     (RCC->APB2RSTR &= ~(RCC_APB2RSTR_IOPGRST))

#define __HAL_RCC_TIM8_FORCE_RESET()        (RCC->APB2RSTR |= (RCC_APB2RSTR_TIM8RST))
#define __HAL_RCC_ADC3_FORCE_RESET()        (RCC->APB2RSTR |= (RCC_APB2RSTR_ADC3RST))

#define __HAL_RCC_TIM8_RELEASE_RESET()      (RCC->APB2RSTR &= ~(RCC_APB2RSTR_TIM8RST))
#define __HAL_RCC_ADC3_RELEASE_RESET()      (RCC->APB2RSTR &= ~(RCC_APB2RSTR_ADC3RST))


/**
  * @}
  */

/** @defgroup RCCEx_HSE_Configuration HSE Configuration
  * @{
  */

/**
  * @brief  Macro to configure the External High Speed oscillator (HSE) Predivision factor for PLL.
  * @note   Predivision factor can not be changed if PLL is used as system clock
  *         In this case, you have to select another source of the system clock, disable the PLL and
  *         then change the HSE predivision factor.
  * @param  __HSE_PREDIV_VALUE__ specifies the division value applied to HSE.
  *         This parameter must be a number between RCC_HSE_PREDIV_DIV1 and RCC_HSE_PREDIV_DIV2.
  */
#define __HAL_RCC_HSE_PREDIV_CONFIG(__HSE_PREDIV_VALUE__) \
                  MODIFY_REG(RCC->CFGR,RCC_CFGR_PLLXTPRE, (uint32_t)(__HSE_PREDIV_VALUE__))

/**
  * @brief  Macro to get prediv1 factor for PLL.
  */
#define __HAL_RCC_HSE_GET_PREDIV() READ_BIT(RCC->CFGR, RCC_CFGR_PLLXTPRE)

/** @defgroup RCCEx_Peripheral_Configuration Peripheral Configuration
  * @brief  Macros to configure clock source of different peripherals.
  * @{
  */

/** @brief  Macro to configure the USB clock.
  * @param  __USBCLKSOURCE__ specifies the USB clock source.
  *          This parameter can be one of the following values:
  *            @arg @ref RCC_USBCLKSOURCE_PLL PLL clock divided by 1 selected as USB clock
  *            @arg @ref RCC_USBCLKSOURCE_PLL_DIV1_5 PLL clock divided by 1.5 selected as USB clock
  */
#define __HAL_RCC_USB_CONFIG(__USBCLKSOURCE__) \
                  MODIFY_REG(RCC->CFGR, RCC_CFGR_USBPRE, (uint32_t)(__USBCLKSOURCE__))

/** @brief  Macro to get the USB clock (USBCLK).
  * @retval The clock source can be one of the following values:
  *            @arg @ref RCC_USBCLKSOURCE_PLL PLL clock divided by 1 selected as USB clock
  *            @arg @ref RCC_USBCLKSOURCE_PLL_DIV1_5 PLL clock divided by 1.5 selected as USB clock
  */
#define __HAL_RCC_GET_USB_SOURCE() ((uint32_t)(READ_BIT(RCC->CFGR, RCC_CFGR_USBPRE)))

/** @brief  Macro to configure the ADCx clock (x=1 to 3 depending on devices).
  * @param  __ADCCLKSOURCE__ specifies the ADC clock source.
  *          This parameter can be one of the following values:
  *            @arg @ref RCC_ADCPCLK2_DIV2 PCLK2 clock divided by 2 selected as ADC clock
  *            @arg @ref RCC_ADCPCLK2_DIV4 PCLK2 clock divided by 4 selected as ADC clock
  *            @arg @ref RCC_ADCPCLK2_DIV6 PCLK2 clock divided by 6 selected as ADC clock
  *            @arg @ref RCC_ADCPCLK2_DIV8 PCLK2 clock divided by 8 selected as ADC clock
  */
#define __HAL_RCC_ADC_CONFIG(__ADCCLKSOURCE__) \
                  MODIFY_REG(RCC->CFGR, RCC_CFGR_ADCPRE, (uint32_t)(__ADCCLKSOURCE__))

/** @brief  Macro to get the ADC clock (ADCxCLK, x=1 to 3 depending on devices).
  * @retval The clock source can be one of the following values:
  *            @arg @ref RCC_ADCPCLK2_DIV2 PCLK2 clock divided by 2 selected as ADC clock
  *            @arg @ref RCC_ADCPCLK2_DIV4 PCLK2 clock divided by 4 selected as ADC clock
  *            @arg @ref RCC_ADCPCLK2_DIV6 PCLK2 clock divided by 6 selected as ADC clock
  *            @arg @ref RCC_ADCPCLK2_DIV8 PCLK2 clock divided by 8 selected as ADC clock
  */
#define __HAL_RCC_GET_ADC_SOURCE() ((uint32_t)(READ_BIT(RCC->CFGR, RCC_CFGR_ADCPRE)))


/* Exported functions --------------------------------------------------------*/
/** @addtogroup RCCEx_Exported_Functions
  * @{
  */

/** @addtogroup RCCEx_Exported_Functions_Group1
  * @{
  */

HAL_StatusTypeDef HAL_RCCEx_PeriphCLKConfig(RCC_PeriphCLKInitTypeDef  *PeriphClkInit);
void              HAL_RCCEx_GetPeriphCLKConfig(RCC_PeriphCLKInitTypeDef  *PeriphClkInit);
uint32_t          HAL_RCCEx_GetPeriphCLKFreq(uint32_t PeriphClk);

#ifdef __cplusplus
}
#endif

#endif /* __STM32F1xx_HAL_RCC_EX_H */
