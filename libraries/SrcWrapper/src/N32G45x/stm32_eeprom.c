/**
  ******************************************************************************
  * @file    stm32_eeprom.c
  * @brief   Provides emulated eeprom from flash
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2016 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

#include "stm32_eeprom.h"
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Be able to change FLASH_BANK_NUMBER to use if relevant */
#if !defined(FLASH_BANK_NUMBER)
#define FLASH_BANK_NUMBER   FLASH_BANK_1
#endif /* !FLASH_BANK_NUMBER */

/* Be able to change FLASH_END to use */
#define FLASH_END  FLASH_BANK1_END

/* Be able to change FLASH_BASE_ADDRESS to use */
/*
 * By default, Use the last page of the flash to store data
 * in order to prevent overwritting
 * program data
 */

#define FLASH_BASE_ADDRESS  ((uint32_t)((FLASH_END + 1) - FLASH_PAGE_SIZE))

static uint8_t eeprom_buffer[E2END + 1] __attribute__((aligned(8))) = {0};

/**
  * @brief  Function reads a byte from emulated eeprom (flash)
  * @param  pos : address to read
  * @retval byte : data read from eeprom
  */
uint8_t eeprom_read_byte(const uint32_t pos)
{
  eeprom_buffer_fill();
  return eeprom_buffered_read_byte(pos);
}

/**
  * @brief  Function writes a byte to emulated eeprom (flash)
  * @param  pos : address to write
  * @param  value : value to write
  * @retval none
  */
void eeprom_write_byte(uint32_t pos, uint8_t value)
{
  eeprom_buffered_write_byte(pos, value);
  eeprom_buffer_flush();
}

/**
  * @brief  Function reads a byte from the eeprom buffer
  * @param  pos : address to read
  * @retval byte : data read from eeprom
  */
uint8_t eeprom_buffered_read_byte(const uint32_t pos)
{
  return eeprom_buffer[pos];
}

/**
  * @brief  Function writes a byte to the eeprom buffer
  * @param  pos : address to write
  * @param  value : value to write
  * @retval none
  */
void eeprom_buffered_write_byte(uint32_t pos, uint8_t value)
{
  eeprom_buffer[pos] = value;
}

/**
  * @brief  This function copies the data from flash into the buffer
  * @param  none
  * @retval none
  */
void eeprom_buffer_fill(void)
{
  memcpy(eeprom_buffer, (uint8_t *)(FLASH_BASE_ADDRESS), E2END + 1);
}

/**
  * @brief  This function writes the buffer content into the flash
  * @param  none
  * @retval none
  */
void eeprom_buffer_flush(void)
{
  FLASH_EraseInitTypeDef EraseInitStruct;
  uint32_t offset = 0;
  uint32_t address = FLASH_BASE_ADDRESS;
  uint32_t address_end = FLASH_BASE_ADDRESS + E2END;
  uint32_t pageError = 0;
  uint64_t data = 0;

  /* ERASING page */
  EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
  EraseInitStruct.PageAddress = FLASH_BASE_ADDRESS;
  EraseInitStruct.NbPages = 1;

  if (HAL_FLASH_Unlock() == HAL_OK) {

    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_WRPERR | FLASH_FLAG_PGERR);

    if (HAL_FLASHEx_Erase(&EraseInitStruct, &pageError) == HAL_OK) {
      while (address <= address_end) {
        data = *((uint64_t *)((uint8_t *)eeprom_buffer + offset));
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, address, data) == HAL_OK) {
          address += 8;
          offset += 8;
        } else {
          address = address_end + 1;
        }
      }
    }
    HAL_FLASH_Lock();
  }
}

#ifdef __cplusplus
}
#endif

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
