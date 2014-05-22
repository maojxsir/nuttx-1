/************************************************************************************
 * configs/superswitch/src/superswitch.c
 * arch/arm/src/board/up_boot.c
 *
 *   Copyright (C) 2009, 2012 Gregory Nutt. All rights reserved.
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
#include "superswitch-internal.h"


/************************************************************************************
 * Global definition
 ************************************************************************************/
static struct stm32_tim_dev_s *tim3 = NULL;
static struct stm32_tim_dev_s *tim4 = NULL;
static struct stm32_tim_dev_s *tim2 = NULL;

void superswitch_timer_init(void)
{
  printf("+superswitch_timer_init\n");
  tim2 = stm32_tim_init(2);
  tim3 = stm32_tim_init(3);
  tim4 = stm32_tim_init(4);
  if (tim3 == NULL || tim4 == NULL) {
      printf("ERROR:superswitch:init tim3/tim4 failed\n");
      return;
  }
  // PERIOD 1S, Compare 0. Default Delay is off
  STM32_TIM_SETPERIOD(tim2,TIM3_CLOCK*3);
  STM32_TIM_SETPERIOD(tim3,TIM3_CLOCK*3);
  STM32_TIM_SETPERIOD(tim4,TIM4_CLOCK*3);
  STM32_TIM_SETCOMPARE(tim2,3,1000);
  STM32_TIM_SETCOMPARE(tim3,3,1100);
  STM32_TIM_SETCOMPARE(tim3,4,1200);
  STM32_TIM_SETCOMPARE(tim4,1,1300);
  STM32_TIM_SETCHANNEL(tim2,3,STM32_TIM_CH_OUTPWM | STM32_TIM_CH_POLARITY_POS);
  STM32_TIM_SETCHANNEL(tim3,3,STM32_TIM_CH_OUTPWM | STM32_TIM_CH_POLARITY_POS);
  STM32_TIM_SETCHANNEL(tim3,4,STM32_TIM_CH_OUTPWM | STM32_TIM_CH_POLARITY_POS);
  STM32_TIM_SETCHANNEL(tim4,1,STM32_TIM_CH_OUTPWM | STM32_TIM_CH_POLARITY_POS);
  STM32_TIM_SETMODE(tim2, STM32_TIM_MODE_UP);
  STM32_TIM_SETMODE(tim3, STM32_TIM_MODE_UP);
  STM32_TIM_SETMODE(tim4, STM32_TIM_MODE_UP);
  STM32_TIM_SETCLOCK(tim2,TIM3_CLOCK);
  STM32_TIM_SETCLOCK(tim3,TIM3_CLOCK);
  STM32_TIM_SETCLOCK(tim4,TIM4_CLOCK);
  printf("-superswitch_timer_init\n");
}


