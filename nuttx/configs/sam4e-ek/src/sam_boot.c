/************************************************************************************
 * configs/sam4e-ek/src/sam_boot.c
 *
 *   Copyright (C) 2014 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ************************************************************************************/

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include <debug.h>

#include <arch/board/board.h>

#include "up_arch.h"
#include "sam4e-ek.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/************************************************************************************
 * Private Functions
 ************************************************************************************/

/************************************************************************************
 * Name: board_config_usart1
 *
 * Description:
 *   USART1: To avoid any electrical conflict, the RS232 and RS485 transceiver are
 *   isolated from the receiving line PA21.
 *
 *   - Chose RS485 channel: Close 1-2 pins on JP11 and set PA23 to high level
 *   - Chose RS232 channel: Close 2-3 pins on JP11 and set PA23 to low level
 *
 ************************************************************************************/

#ifdef CONFIG_SAM34_USART1
static inline void board_config_usart1(void)
{
#if defined(CONFIG_USART1_ISUART)
  (void)sam_configgpio(GPIO_RS232_ENABLE);
#else /* if defined(CONFIG_USART1_RS485) */
  (void)sam_configgpio(GPIO_RS485_ENABLE);
#endif
}
#else
#  define board_config_usart1()
#endif

/************************************************************************************
 * Public Functions
 ************************************************************************************/

/************************************************************************************
 * Name: sam_boardinitialize
 *
 * Description:
 *   All SAM3U architectures must provide the following entry point.  This entry point
 *   is called early in the initialization -- after all memory has been configured
 *   and mapped but before any devices have been initialized.
 *
 ************************************************************************************/

void sam_boardinitialize(void)
{
  /* Configure USART1 for RS-232/RS-485 operation */

  board_config_usart1();

  /* Configure SPI chip selects if 1) SPI is not disabled, and 2) the weak function
   * sam_spiinitialize() has been brought into the link.
   */

#ifdef CONFIG_SAM34_SPI0
  if (sam_spiinitialize)
    {
      sam_spiinitialize();
    }
#endif

   /* Initialize USB if 1) USBDEV is selected, 2) the USB controller is not
    * disabled, and 3) the weak function sam_usbinitialize() has been brought
    * into the build.
    */

#if defined(CONFIG_USBDEV) && defined(CONFIG_SAM34_USB)
  if (sam_usbinitialize)
    {
      sam_usbinitialize();
    }
#endif

  /* Configure on-board LEDs if LED support has been selected. */

#ifdef CONFIG_ARCH_LEDS
  board_led_initialize();
#endif

  /* Setup SD card-related PIOs if 1) HSMCI is selected and 2) the weak
   * function sam_hsmciinit() has been brought into the build.
    */

#ifdef CONFIG_SAM34_HSMCI
  if (sam_hsmciinit)
    {
      sam_hsmciinit();
    }
#endif
}
