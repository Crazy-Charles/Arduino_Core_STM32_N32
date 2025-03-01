#ifndef _STM32_DEF_BUILD_
#define _STM32_DEF_BUILD_

#if !defined(CMSIS_STARTUP_FILE) && !defined(CUSTOM_STARTUP_FILE)
#define CMSIS_STARTUP_FILE "startup_n32g45x.s"
#else
  #warning "No CMSIS startup file defined, custom one should be used"
#endif /* !CMSIS_STARTUP_FILE && !CUSTOM_STARTUP_FILE */
#endif /* _STM32_DEF_BUILD_ */
