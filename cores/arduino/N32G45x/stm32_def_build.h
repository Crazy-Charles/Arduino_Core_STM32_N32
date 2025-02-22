#ifndef _STM32_DEF_BUILD_
#define _STM32_DEF_BUILD_

#if !defined(CMSIS_STARTUP_FILE) && !defined(CUSTOM_STARTUP_FILE)
  #if defined(STM32F103xE)
    //#define CMSIS_STARTUP_FILE "startup_stm32f103xe.s"
    #define CMSIS_STARTUP_FILE "startup_n32g45x.s"
  #else
    #error UNKNOWN CHIP
  #endif
#else
  #warning "No CMSIS startup file defined, custom one should be used"
#endif /* !CMSIS_STARTUP_FILE && !CUSTOM_STARTUP_FILE */
#endif /* _STM32_DEF_BUILD_ */
