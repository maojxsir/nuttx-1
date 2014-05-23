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

#ifndef __CONFIGS_SUPER_SWITCH_INTERNAL_H
#define __CONFIGS_SUPER_SWITCH_INTERNAL_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>
#include <stdint.h>

/************************************************************************************
 * Definitions
 ************************************************************************************/

/* STM3210E-EVAL GPIOs **************************************************************/
/* LEDs */
/* USB D+ and D- controlled by one same gpio */
#define GPIO_RELAY_USBDATA  (GPIO_OUTPUT|GPIO_CNF_OUTPP|GPIO_MODE_50MHz|\
                             GPIO_OUTPUT_CLEAR|GPIO_PORTB|GPIO_PIN0)

#define GPIO_RELAY_USBVBUS  (GPIO_OUTPUT|GPIO_CNF_OUTPP|GPIO_MODE_50MHz|\
                             GPIO_OUTPUT_CLEAR|GPIO_PORTB|GPIO_PIN1)

#define GPIO_RELAY_POWER_A  (GPIO_OUTPUT|GPIO_CNF_OUTPP|GPIO_MODE_50MHz|\
                             GPIO_OUTPUT_CLEAR|GPIO_PORTB|GPIO_PIN6)

#define GPIO_RELAY_POWER_B  (GPIO_OUTPUT|GPIO_CNF_OUTPP|GPIO_MODE_50MHz|\
                             GPIO_OUTPUT_CLEAR|GPIO_PORTD|GPIO_PIN2)

/* tim3 min step 1ms */
#define TIM2_CLOCK          (1000) 
/* tim3 min step 1ms */
#define TIM3_CLOCK          (TIM2_CLOCK)  
/* tim4 min step 1ms */
#define TIM4_CLOCK          (TIM2_CLOCK)
 
/************************************************************************************
 * Public Types
 ************************************************************************************/
typedef struct {
  char  *name;
  int    channel; 
  xcpt_t isr;
  void  *priv;
}relay_info;

/************************************************************************************
 * Public data
 ************************************************************************************/

#ifndef __ASSEMBLY__

void superswitch_gpioinit(void);
void superswitch_timer_init(void);

#endif /* __ASSEMBLY__ */
#endif /* __CONFIGS_SUPER_SWITCH_INTERNAL_H */

