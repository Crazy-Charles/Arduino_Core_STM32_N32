#ifndef __N32G45x_HAL_CONF_H
#define __N32G45x_HAL_CONF_H

#include "variant.h"

/* STM32F1xx specific HAL configuration options. */
#if __has_include("hal_conf_custom.h")
#include "hal_conf_custom.h"
#else
#if __has_include("hal_conf_extra.h")
#include "hal_conf_extra.h"
#endif
#include "n32g45x_hal_conf_default.h"
#endif

#endif /* __N32G45x_HAL_CONF_H was __STM32F1xx_HAL_CONF_H */
