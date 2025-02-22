/*****************************************************************************
 * Copyright (c) 2019, Nations Technologies Inc.
 *
 * All rights reserved.
 * ****************************************************************************
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * - Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the disclaimer below.
 *
 * Nations' name may not be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * DISCLAIMER: THIS SOFTWARE IS PROVIDED BY NATIONS "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * DISCLAIMED. IN NO EVENT SHALL NATIONS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * ****************************************************************************/

/**
 * @file system_n32g45x.c
 * @author Nations
 * @version v1.0.4
 *
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */

#include "stm32f1xx.h"
#include <stdbool.h>

/*!< Uncomment the following line if you need to use external SRAM  */
/* #define DATA_IN_ExtSRAM */


#define SYSCLK_USE_HSI     0
#define SYSCLK_USE_HSE     1
#define SYSCLK_USE_HSI_PLL 2
#define SYSCLK_USE_HSE_PLL 3

#ifndef SYSCLK_FREQ
  #define SYSCLK_FREQ 144000000
#endif

#if !defined HSE_VALUE
  #define HSE_VALUE (8000000) /*!< Value of the External oscillator in Hz */
#endif                        /* HSE_VALUE */

#if !defined HSI_VALUE
  #define HSI_VALUE (8000000) /*!< Value of the Internal oscillator in Hz */
#endif

#ifndef SYSCLK_SRC
  #define SYSCLK_SRC SYSCLK_USE_HSE_PLL
#endif

#if SYSCLK_SRC == SYSCLK_USE_HSI

#if SYSCLK_FREQ != HSI_VALUE
  #error SYSCL_FREQ must be set to HSI_VALUE
#endif

#elif SYSCLK_SRC == SYSCLK_USE_HSE

#ifndef HSE_VALUE
  #error HSE_VALUE must be defined!
#endif

#if SYSCLK_FREQ != HSE_VALUE
  #error SYSCL_FREQ must be set to HSE_VALUE
#endif

#elif SYSCLK_SRC == SYSCLK_USE_HSI_PLL

#if (SYSCLK_FREQ < 32000000)
  #error When SYSCL_FREQ is PLL, SYSCL_FREQ must be greater than 32MHZ
#endif

#if (SYSCLK_FREQ % (HSI_VALUE / 2) == 0) && (SYSCLK_FREQ / (HSI_VALUE / 2) >= 2)                                       \
    && (SYSCLK_FREQ / (HSI_VALUE / 2) <= 32)

  #define PLLSRC_DIV 2
  #define PLL_MUL    (SYSCLK_FREQ / (HSI_VALUE / 2))
  
#else
  #error Cannot make a PLL multiply factor to SYSCLK_FREQ.
#endif

#elif SYSCLK_SRC == SYSCLK_USE_HSE_PLL

#if (SYSCLK_FREQ < 32000000)
  #error When SYSCL_FREQ is PLL, SYSCL_FREQ must be greater than 32MHZ
#endif

#ifndef HSE_VALUE
  #error HSE_VALUE must be defined!
#endif

#if ((SYSCLK_FREQ % (HSE_VALUE / 2)) == 0) && (SYSCLK_FREQ / (HSE_VALUE / 2) >= 2)                                     \
    && (SYSCLK_FREQ / (HSE_VALUE / 2) <= 32)

  #define PLLSRC_DIV 2
  #define PLL_MUL    (SYSCLK_FREQ / (HSE_VALUE / 2))

#elif (SYSCLK_FREQ % HSE_VALUE == 0) && (SYSCLK_FREQ / HSE_VALUE >= 2) && (SYSCLK_FREQ / HSE_VALUE <= 32)

  #define PLLSRC_DIV 1
  #define PLL_MUL    (SYSCLK_FREQ / HSE_VALUE)

#else
  #error Cannot make a PLL multiply factor to SYSCLK_FREQ.
#endif

#else
  #error wrong value for SYSCLK_SRC
#endif

/*!< Uncomment the following line if you need to relocate your vector Table in
     Internal SRAM. */
/* #define VECT_TAB_SRAM */
#ifndef VECT_TAB_OFFSET
#define VECT_TAB_OFFSET  0x00000000U /*!< Vector Table base offset field.
                                  This value must be a multiple of 0x200. */
#endif

  /* This variable is updated in three ways:
      1) by calling CMSIS function SystemCoreClockUpdate()
      2) by calling HAL API function HAL_RCC_GetHCLKFreq()
      3) each time HAL_RCC_ClockConfig() is called to configure the system clock frequency
         Note: If you use this function to configure the system clock; then there
               is no need to call the 2 first functions listed above, since SystemCoreClock
               variable is updated automatically.
  */
uint32_t SystemCoreClock = SYSCLK_FREQ;

const uint8_t AHBPrescTable[16U] = {0, 0, 0, 0, 0, 0, 0, 0, 1, 2, 3, 4, 6, 7, 8, 9};
const uint8_t APBPrescTable[8U] =  {0, 0, 0, 0, 1, 2, 3, 4};


static void SetSysClock(void);
static void SetVDDDBKTrim(void);

#ifdef DATA_IN_ExtSRAM
  static void SystemInit_ExtMemCtl(void);
#endif /* DATA_IN_ExtSRAM */

/**
 * @brief  Setup the microcontroller system
 *         Initialize the Embedded Flash Interface, the PLL and update the
 *         SystemCoreClock variable.
 * @note   This function should be used only after reset.
 */

 void SystemInit(void)
 {
     /* FPU settings
      * ------------------------------------------------------------*/
 #if (__FPU_PRESENT == 1) && (__FPU_USED == 1)
     SCB->CPACR |= ((3UL << 10 * 2) | (3UL << 11 * 2)); /* set CP10 and CP11 Full Access */
 #endif
 
     /* Reset the RCC clock configuration to the default reset state(for debug purpose) */
     /* Set HSIEN bit */
     RCC->CR |= (uint32_t)0x00000001;
 
     /* Reset SW, HPRE, PPRE1, PPRE2, ADCPRE and MCO bits */
     RCC->CFGR &= (uint32_t)0xF8FFC000;
 
     /* Reset HSEON, CLKSSEN and PLLEN bits */
     RCC->CR &= (uint32_t)0xFEF6FFFF;
 
     /* Reset HSEBYP bit */
     RCC->CR &= (uint32_t)0xFFFBFFFF;
 
     /* Reset PLLSRC, PLLXTPRE, PLLMUL and USBPRES/OTGFSPRE bits */
     RCC->CFGR &= (uint32_t)0xF700FFFF;
 
     /* Reset CFG2 register */
     RCC->CFG2 = 0x00003800;
 
     /* Reset CFG3 register */
     RCC->CFG3 = 0x00003840;
 
     /* Disable all interrupts and clear pending bits  */
     RCC->CIR = 0x009F0000;
 
     /* Enable ex mode */
     RCC->APB1ENR |= ((uint32_t)0x10000000);
     PWR->CTRL3 |= 0x00000001;
     RCC->APB1ENR &= (uint32_t)(~((uint32_t)0x10000000));
 
     /* Enable ICACHE and Prefetch Buffer */
      FLASH->ACR |= (uint32_t)(0x80 | 0x10);
 
 #ifdef DATA_IN_ExtSRAM
     SystemInit_ExtMemCtl();
 #endif /* DATA_IN_ExtSRAM */
 
     /* Configure the System clock frequency, HCLK, PCLK2 and PCLK1 prescalers */
     /* Configure the Flash Latency cycles and enable prefetch buffer */
     SetSysClock();
 
 #ifdef VECT_TAB_SRAM
     SCB->VTOR = SRAM_BASE | VECT_TAB_OFFSET; /* Vector Table Relocation in Internal SRAM. */
 #else
     SCB->VTOR = FLASH_BASE | VECT_TAB_OFFSET; /* Vector Table Relocation in Internal FLASH. */
 #endif
     /*cheak VDDDBKR Increase flag */
     if((*(__IO uint32_t*)(0x1FFFF298)) != 0x49524B42){
         SetVDDDBKTrim();        
     }
 }

/**
 * @brief  Update SystemCoreClock variable according to Clock Register Values.
 *         The SystemCoreClock variable contains the core clock (HCLK), it can
 *         be used by the user application to setup the SysTick timer or
 * configure other parameters.
 *
 * @note   Each time the core clock (HCLK) changes, this function must be called
 *         to update SystemCoreClock variable value. Otherwise, any
 * configuration based on this variable will be incorrect.
 *
 * @note   - The system frequency computed by this function is not the real
 *           frequency in the chip. It is calculated based on the predefined
 *           constant and the selected clock source:
 *
 *           - If SYSCLK source is HSI, SystemCoreClock will contain the
 * HSI_VALUE(*)
 *
 *           - If SYSCLK source is HSE, SystemCoreClock will contain the
 * HSE_VALUE(**)
 *
 *           - If SYSCLK source is PLL, SystemCoreClock will contain the
 * HSE_VALUE(**) or HSI_VALUE(*) multiplied by the PLL factors.
 *
 *         (*) HSI_VALUE is a constant defined in n32g45x.h file (default value
 *             8 MHz) but the real value may vary depending on the variations
 *             in voltage and temperature.
 *
 *         (**) HSE_VALUE is a constant defined in N32G45X.h file (default value
 *              8 MHz or 25 MHz, depedning on the product used), user has to
 * ensure that HSE_VALUE is same as the real frequency of the crystal used.
 *              Otherwise, this function may have wrong result.
 *
 *         - The result of this function could be not correct when using
 * fractional value for HSE crystal.
 */

 void SystemCoreClockUpdate(void)
{
    uint32_t tmp = 0, pllmull = 0, pllsource = 0;

    /* Get SYSCLK source
     * -------------------------------------------------------*/
    tmp = RCC->CFGR & ((uint32_t)0x0000000C);

    switch (tmp)
    {
    case 0x00: /* HSI used as system clock */
        SystemCoreClock = HSI_VALUE;
        break;
    case 0x04: /* HSE used as system clock */
        SystemCoreClock = HSE_VALUE;
        break;
    case 0x08: /* PLL used as system clock */

        /* Get PLL clock source and multiplication factor
         * ----------------------*/
        pllmull   = RCC->CFGR & ((uint32_t)0x083C0000);
        pllsource = RCC->CFGR & ((uint32_t)0x00010000);

        if ((pllmull & ((uint32_t)0x08000000)) == 0)
        {
            pllmull = (pllmull >> 18) + 2; // PLLMUL[4]=0
        }
        else
        {
            pllmull = ((pllmull >> 18) - 496) + 1; // PLLMUL[4]=1
        }

        if (pllsource == 0x00)
        {
            /* HSI oscillator clock divided by 2 selected as PLL clock entry */
            SystemCoreClock = (HSI_VALUE >> 1) * pllmull;
        }
        else
        {
            /* HSE selected as PLL clock entry */
            if ((RCC->CFGR & ((uint32_t)0x00020000)) != (uint32_t)RESET)
            { /* HSE oscillator clock divided by 2 */
                SystemCoreClock = (HSE_VALUE >> 1) * pllmull;
            }
            else
            {
                SystemCoreClock = HSE_VALUE * pllmull;
            }
        }

        break;

    default:
        SystemCoreClock = HSI_VALUE;
        break;
    }

    /* Compute HCLK clock frequency ----------------*/
    /* Get HCLK prescaler */
    tmp = AHBPrescTable[((RCC->CFGR & ((uint32_t)0x000000F0)) >> 4)];
    /* HCLK clock frequency */
    SystemCoreClock >>= tmp;
}


/**
 * @brief  Configures the System clock frequency, HCLK, PCLK2 and PCLK1
 * prescalers.
 */
static void SetSysClock(void)
{
    volatile uint32_t rcc_cfgr       = 0;
    volatile bool HSEStatus          = 0;
    volatile uint32_t StartUpCounter = 0;
    uint32_t tmpregister    = (uint32_t)0x00;
#if SYSCLK_SRC == SYSCLK_USE_HSE || SYSCLK_SRC == SYSCLK_USE_HSE_PLL

    /* Enable HSE */
    RCC->CR |= ((uint32_t)0x00010000);

    /* Wait till HSE is ready and if Time out is reached exit */
    do
    {
        HSEStatus = RCC->CR & ((uint32_t)0x00020000);
        StartUpCounter++;
    } while ((HSEStatus == 0) && (StartUpCounter != ((uint16_t)0x8000)));

    HSEStatus = ((RCC->CR & ((uint32_t)0x00020000)) != RESET);
    if (!HSEStatus)
    {
        /* If HSE fails to start-up, the application will have wrong clock
         * configuration. User can add here some code to deal with this error */
        SystemCoreClock = HSI_VALUE;
        return;
    }
#endif

    /* Flash wait state
        0: HCLK <= 32M
        1: HCLK <= 64M
        2: HCLK <= 96M
        3: HCLK <= 128M
        4: HCLK <= 144M
     */
    tmpregister = FLASH->ACR;
    tmpregister &= (uint32_t)((uint32_t)~((uint8_t)0x07));
    tmpregister |= (uint32_t)((SYSCLK_FREQ - 1) / 32000000);
    FLASH->ACR = tmpregister;
    /* HCLK = SYSCLK */
    RCC->CFGR |= (uint32_t)0x00000000;

    /* PCLK2 max 72M */
    if (SYSCLK_FREQ > 72000000)
    {
        RCC->CFGR |= (uint32_t)0x00002000;
    }
    else
    {
        RCC->CFGR |= (uint32_t)0x00000000;
    }

    /* PCLK1 max 36M */
    if (SYSCLK_FREQ > 72000000)
    {
        RCC->CFGR |= (uint32_t)0x00000500;
    }
    else if (SYSCLK_FREQ > 36000000)
    {
        RCC->CFGR |= (uint32_t)0x00000400;
    }
    else
    {
        RCC->CFGR |= (uint32_t)0x00000000;
    }

#if SYSCLK_SRC == SYSCLK_USE_HSE
    /* Select HSE as system clock source */
    RCC->CFG &= (uint32_t)((uint32_t) ~(RCC_CFG_SCLKSW));
    RCC->CFG |= (uint32_t)RCC_CFG_SCLKSW_HSE;

    /* Wait till HSE is used as system clock source */
    while ((RCC->CFG & (uint32_t)RCC_CFG_SCLKSTS) != (uint32_t)0x04)
    {
    }
#elif SYSCLK_SRC == SYSCLK_USE_HSI_PLL || SYSCLK_SRC == SYSCLK_USE_HSE_PLL
    tmpregister = (uint32_t)0x00;
    tmpregister = RCC->CFGR;
    /* clear bits */
    tmpregister &= (uint32_t)((uint32_t) ~(((uint32_t)0x00010000) | ((uint32_t)0x00020000) | ((uint32_t)0x083C0000)));

    /* set PLL source */
    rcc_cfgr = tmpregister;
    rcc_cfgr |= (SYSCLK_SRC == SYSCLK_USE_HSI_PLL ? ((uint32_t)0x00000000) : ((uint32_t)0x00010000));

#if SYSCLK_SRC == SYSCLK_USE_HSE_PLL
    rcc_cfgr |= (PLLSRC_DIV == 1 ? ((uint32_t)0x00000000) : ((uint32_t)0x00020000));
#endif

    /* set PLL multiply factor */
#if PLL_MUL <= 16
    rcc_cfgr |= (PLL_MUL - 2) << 18;
#else
    rcc_cfgr |= ((PLL_MUL - 17) << 18) | (1 << 27);
#endif

    RCC->CFGR = rcc_cfgr;

    /* Enable PLL */
    RCC->CR |= ((uint32_t)0x01000000);

    /* Wait till PLL is ready */
    while ((RCC->CR & ((uint32_t)0x02000000)) == 0)
    {
    }

    tmpregister = (uint32_t)0x00;
    tmpregister = RCC->CFGR;
    /* Select PLL as system clock source */
    tmpregister &= (uint32_t)((uint32_t) ~(0x00000003));
    tmpregister |= (uint32_t)0x00000002;
    RCC->CFGR = tmpregister;

    /* Wait till PLL is used as system clock source */
    while ((RCC->CFGR & (uint32_t)0x0000000C) != (uint32_t)0x08)
    {
    }
#endif
}
/**
 * @brief  Configures the VDDDBK voltage
 * 
 */
static void SetVDDDBKTrim(void)
{
    uint32_t tmpregister = 0, tmp_VDDDBK_trim_value = 0;

    /* Enable PWR clock */
    RCC->APB1ENR |= (uint32_t)0x10000000;

    //get default VDDDBK trim value
    tmp_VDDDBK_trim_value = *(__IO uint32_t*)(0x40001800);
    tmp_VDDDBK_trim_value = (tmp_VDDDBK_trim_value >> 24) & 0x1F;
    
    tmpregister = PWR->CTRL3;
    //clear set VDDDBK trim value
    tmpregister &= (~0x3E0);
    
    //VDDDBK + 2
    tmp_VDDDBK_trim_value = tmp_VDDDBK_trim_value + 2;

    //write value
    if((tmp_VDDDBK_trim_value > 0) && (tmp_VDDDBK_trim_value < 0x1F)){
        tmpregister |= tmp_VDDDBK_trim_value << 5;
        PWR->CTRL3 =  tmpregister;
    }else{
        tmpregister |= 0x1F << 5;
        PWR->CTRL3 =  tmpregister;
    }
    /* Disable PWR clock */
    RCC->APB1ENR &= ~((uint32_t)0x10000000);
}
