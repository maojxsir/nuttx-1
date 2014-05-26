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


static int superswitch_tim2_isr(int irq, void *context);
static int superswitch_tim3_isr(int irq, void *context);
static int superswitch_tim4_isr(int irq, void *context);

/************************************************************************************
 * Global definition
 ************************************************************************************/
static struct stm32_tim_dev_s *tim3 = NULL;
static struct stm32_tim_dev_s *tim4 = NULL;
static struct stm32_tim_dev_s *tim2 = NULL;

static relay_param powera_param = {0};
static relay_param powerb_param = {0};          

static relay_info g_superswitch_relayinfos[] = {
  {"USB-DATA",3, superswitch_tim3_isr, NULL}, //TIM3
  {"USB-VBUS",4, superswitch_tim3_isr, NULL}, //TIM3
  {"POWER-A", 1, superswitch_tim4_isr, NULL}, //TIM4
  {"POWER-B", 3, superswitch_tim2_isr, NULL}, //TIM2
};
static int g_num_relay = sizeof(g_superswitch_relayinfos)/sizeof(relay_info);

/************************************************************************************
 * Function
 ************************************************************************************/

void superswitch_timer_init(void)
{
  
  printf("+superswitch_timer_init\n");
  tim2 = stm32_tim_init(2);
  tim3 = stm32_tim_init(3);
  tim4 = stm32_tim_init(4);
  if (tim3 == NULL || tim4 == NULL || tim2 == NULL) {
      printf("ERROR:superswitch:init tim3/tim4 failed\n");
      return;
  }
  
  g_superswitch_relayinfos[0].priv = tim3;
  g_superswitch_relayinfos[0].isr = superswitch_tim3_isr;
  g_superswitch_relayinfos[1].priv = tim3;
  g_superswitch_relayinfos[1].isr = superswitch_tim3_isr;
  g_superswitch_relayinfos[2].priv = tim4;
  g_superswitch_relayinfos[2].isr = superswitch_tim4_isr;
  g_superswitch_relayinfos[3].priv = tim2;
  g_superswitch_relayinfos[3].isr = superswitch_tim2_isr;
  

  // PERIOD 1S, Compare 0. Default Delay is off
  STM32_TIM_SETPERIOD(tim2,TIM3_CLOCK);
  STM32_TIM_SETPERIOD(tim3,TIM3_CLOCK);
  STM32_TIM_SETPERIOD(tim4,TIM4_CLOCK);
  STM32_TIM_SETCOMPARE(tim2,3,0);
  STM32_TIM_SETCOMPARE(tim3,3,0);
  STM32_TIM_SETCOMPARE(tim3,4,0);
  STM32_TIM_SETCOMPARE(tim4,1,0);
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
  stm32_tim_disable(tim2);
  stm32_tim_disable(tim3);
  stm32_tim_disable(tim4);
  printf("-superswitch_timer_init\n");
}

static int superswitch_tim2_isr(int irq, void *context)
{
  static int curIndex = 1;
  
  STM32_TIM_ACKINT(tim2,3);
  STM32_TIM_SETPERIOD(tim2,powerb_param.data[curIndex].length);
  STM32_TIM_SETCOMPARE(tim2,3,powerb_param.data[curIndex].high);
  curIndex ++;
  if (curIndex == powerb_param.count) {
    curIndex = 0;
  }
}

static int superswitch_tim3_isr(int irq, void *context)
{
  printf("tim3");
  STM32_TIM_ACKINT(tim3,3);
  STM32_TIM_ACKINT(tim3,4);
}

static int superswitch_tim4_isr(int irq, void *context)
{
  STM32_TIM_ACKINT(tim4,1);
  static int curIndex = 1;
  
  STM32_TIM_ACKINT(tim4,1);
  STM32_TIM_SETPERIOD(tim4,powera_param.data[curIndex].length);
  STM32_TIM_SETCOMPARE(tim4,1,powera_param.data[curIndex].high);
  curIndex ++;
  if (curIndex == powera_param.count) {
      curIndex = 0;
    }
}

int cmd_relay(FAR struct nsh_vtbl_s *vtbl, int argc, char **argv)
{
  int i,channel,length=0,high=0;
  struct stm32_tim_dev_s *tim = NULL;
  xcpt_t isr = NULL;
   
  for (i=0; i<g_num_relay; i++ ) {
    if (strcmp(argv[1],g_superswitch_relayinfos[i].name) == 0) {
      tim = g_superswitch_relayinfos[i].priv;
      channel = g_superswitch_relayinfos[i].channel;
      isr = g_superswitch_relayinfos[i].isr;
    }
  }

  if ( (tim==NULL) ) {
    printf("ERROR:invalid relay id\n");
    return -1;
  }

  length = strtol(argv[2],NULL,10);
  high   = strtol(argv[3],NULL,10);
  if ((length < 0) || (high < 0)) {
      printf("ERROR:invalid length or high period\n");
      return -2;
  }

  if (length != 0) {
    STM32_TIM_SETPERIOD(tim,length);
  } else {
    stm32_tim_disable(tim);
    printf("SUCCESS\n");
    return 0;
  }
  STM32_TIM_SETCOMPARE(tim,channel,high);
  stm32_tim_enable(tim);
  printf("SUCCESS\n");
  return 0;
}

int cmd_sync_relay(FAR struct nsh_vtbl_s *vtbl, int argc, char **argv)
{
  int length_a=0,length_b=0,high_a=0,high_b=0;
  int reg=0;
  
  stm32_tim_disable(tim2);
  stm32_tim_disable(tim4);
  stm32_tim_disable(tim3);
  
  length_a = strtol(argv[1],NULL,10);
  high_a   = strtol(argv[2],NULL,10);
  length_b = strtol(argv[3],NULL,10);
  high_b   = strtol(argv[4],NULL,10);
  
  if ((length_a < 0) || (high_a < 0) || (length_b < 0 ) || (high_b < 0 )) {
    printf("ERROR:invalid length or high period\n");
    return -1;
  }
  
  /* config tim3 as master */
  reg = getreg16(STM32_TIM3_CR2);
  reg &= ~ATIM_CR2_MMS_MASK;
  reg |= ATIM_CR2_MMS_ENABLE;
  putreg16(reg,STM32_TIM3_CR2);
  
  //reg = getreg16(STM32_TIM3_SMCR);
  //reg &= ~ATIM_SMCR_TS_MASK;
  //reg |= ATIM_SMCR_ETRF;
  
  reg &= ~ATIM_SMCR_SMS_MASK;
  reg |= ATIM_SMCR_TRIGGER;
  
  reg |= ATIM_SMCR_MSM;
  putreg16(reg, STM32_TIM3_SMCR);
  
  /* config tim2 and tim4 in slave mode */
  reg = getreg16(STM32_TIM2_SMCR);
  reg &= ~ATIM_SMCR_TS_MASK;
  reg |= ATIM_SMCR_ITR2;
  reg &= ~ATIM_SMCR_SMS_MASK;
  reg |= ATIM_SMCR_TRIGGER;
  
  reg = getreg16(STM32_TIM4_SMCR);
  reg &= ~ATIM_SMCR_TS_MASK;
  reg |= ATIM_SMCR_ITR2;
  reg &= ~ATIM_SMCR_SMS_MASK;
  reg |= ATIM_SMCR_TRIGGER;
  
  STM32_TIM_SETPERIOD(tim4,length_a);
  STM32_TIM_SETPERIOD(tim2,length_b);
  STM32_TIM_SETCOMPARE(tim4, 1, high_a);
  STM32_TIM_SETCOMPARE(tim2, 3, high_b);
  
  stm32_tim_enable(tim2);
  stm32_tim_enable(tim4);
  stm32_tim_enable(tim3);

  return 0;
}

int cmd_relay_param(FAR struct nsh_vtbl_s *vtbl, int argc, char **argv)
{
  int length=0,high=0,index=0;
  relay_param *param = NULL;
  
  if (strcmp(argv[1],"POWER-A") == 0) {
    param = &powera_param;  
  } else if (strcmp(argv[1],"POWER-B") == 0) {
    param = &powerb_param;
  } else {
    printf("ERROR:unsupport ID\n");
    return -1;
  }
  
  index  = strtol(argv[2],NULL,10);
  length = strtol(argv[3],NULL,10);
  high   = strtol(argv[4],NULL,10);
  
  if (index < 0 || index > 50 || length < 0 || high < 0) {
    printf("ERROR:invalid param\n");
    return -2;
  }

  param->data[index].length = length;
  param->data[index].high   = high;

  return 0;
}

int cmd_relay_intr(FAR struct nsh_vtbl_s *vtbl, int argc, char **argv) 
{
  int count=0,channel=0;; 
  relay_param *param = NULL;
  xcpt_t isr = NULL;
  struct stm32_tim_dev_s *tim = NULL;
  
  if (strcmp(argv[1],"POWER-A") == 0) {
      param = &powera_param;  
      isr = superswitch_tim4_isr;
      tim = tim4;
      channel = 1;
  } else if (strcmp(argv[1],"POWER-B") == 0) {
      param = &powerb_param;
      isr = superswitch_tim2_isr;
      tim = tim2;
      channel = 3;
  } else {
      printf("ERROR:unsupport ID\n");
      return -1;
  }

  count = strtol(argv[2],NULL,10);
  
  if (count < 0) {
    printf("ERROR:invalid count number\n");
    return -1;
  }

  param->count = count;
  if (count == 0) {
    STM32_TIM_SETISR(tim,0,channel);
    STM32_TIM_DISABLEINT(tim,channel);
    stm32_tim_disable(tim);
  } else {
    STM32_TIM_SETISR(tim,isr,channel);
    STM32_TIM_ENABLEINT(tim,channel);
    //STM32_TIM_SETPERIOD(tim,param->data[0].length);
    //STM32_TIM_SETCOMPARE(tim,channel,param->data[0].high);
    //stm32_tim_enable(tim);
  }
  
  return 0;
}
