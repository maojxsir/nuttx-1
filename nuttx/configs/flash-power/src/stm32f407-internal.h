/************************************************************************************
 * configs/stm3210e_eval/src/stm3210e_internal.h
 * arch/arm/src/board/stm3210e_internal.n
 *
 *   Copyright (C) 2009 Gregory Nutt. All rights reserved.
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

#ifndef __CONFIGS_FLASH_POWER_INTERNAL_H
#define __CONFIGS_FLASH_POWER_INTERNAL_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>
#include <stdint.h>

/************************************************************************************
 * Definitions
 ************************************************************************************/

/* FlashPower    GPIOs **************************************************************/
/* LEDs */

#define GPIO_FAN    (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|\
                         GPIO_OUTPUT_CLEAR|GPIO_PORTE | GPIO_PIN2)

#define GPIO_RL2V (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_50MHz | \
	                           GPIO_OUTPUT_CLEAR | GPIO_PORTE | GPIO_PIN3)
#define GPIO_RL4V    (GPIO_OUTPUT |GPIO_PUSHPULL | GPIO_SPEED_50MHz | \
	                            GPIO_OUTPUT_CLEAR | GPIO_PORTE | GPIO_PIN4)
#define GPIO_RL8V    (GPIO_OUTPUT |GPIO_PUSHPULL | GPIO_SPEED_50MHz | \
	                            GPIO_OUTPUT_CLEAR | GPIO_PORTE | GPIO_PIN5)
#define GPIO_RL16V    (GPIO_OUTPUT |GPIO_PUSHPULL | GPIO_SPEED_50MHz | \
	                            GPIO_OUTPUT_CLEAR | GPIO_PORTE | GPIO_PIN6)	  
	                            
#define GPIO_OUT_EN (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_50MHz | \
	                               GPIO_OUTPUT_SET | GPIO_PORTC | GPIO_PIN15)
#define GPIO_AC_OV    (GPIO_INPUT | GPIO_FLOAT | GPIO_PUSHPULL | \
	                               GPIO_SPEED_50MHz | GPIO_PORTB | GPIO_PIN6)

#define GPIO_OTGFS_VBUS (GPIO_INPUT|GPIO_FLOAT|GPIO_SPEED_100MHz|GPIO_OPENDRAIN|GPIO_PORTA|GPIO_PIN9)
#define GPIO_OTGFS_PWRON (GPIO_OUTPUT|GPIO_FLOAT|GPIO_SPEED_100MHz|GPIO_PUSHPULL|GPIO_PORTC|GPIO_PIN7)

#ifdef CONFIG_USBHOST
#  define GPIO_OTGFS_OVER  (GPIO_INPUT|GPIO_EXTI|GPIO_FLOAT|GPIO_SPEED_100MHz|GPIO_PUSHPULL|GPIO_PORTC|GPIO_PIN6)

#else
#  define GPIO_OTGFS_OVER  (GPIO_INPUT|GPIO_FLOAT|GPIO_SPEED_100MHz|GPIO_PUSHPULL|GPIO_PORTC|GPIO_PIN6)
#endif

/************************************************************************************
 * Public Types
 ************************************************************************************/


/************************************************************************************
 * Public data
 ************************************************************************************/

#ifndef __ASSEMBLY__

/************************************************************************************
 * Public Functions
 ************************************************************************************/


#endif /* __ASSEMBLY__ */
#endif /* __CONFIGS_STM3210E_EVAL_SRC_STM3210E_INTERNAL_H */

