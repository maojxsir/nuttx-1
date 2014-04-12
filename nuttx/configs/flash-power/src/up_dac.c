/************************************************************************************
 * configs/stm3210e-eval/src/up_adc.c
 * arch/arm/src/board/up_adc.c
 *
 *   Copyright (C) 2011-2012 Gregory Nutt. All rights reserved.
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

#include <errno.h>
#include <debug.h>
//#include <nuttx/analog/dac.h>
#include <arch/board/board.h>
#include <chip/stm32_dac.h>
#include "chip.h"
#include "up_arch.h"

#include "stm32f407-internal.h"

FAR struct dac_dev_s *stm32_dacinitialize(int intf);
int dac_register(FAR const char *path, FAR struct dac_dev_s *dev);

#ifdef CONFIG_DAC

int dac_devinit(void)
{
  static bool initialized = false;
  struct dac_dev_s *dac; 
  int ret;
 
  if (!initialized) {
#ifdef CONFIG_STM32_DAC1
    dac = stm32_dacinitialize(1);
    if (NULL == dac) {
      adbg("ERROR:Failed to get DAC0 interface\n");
	    return -ENODEV;
    }
    ret = dac_register("/dev/dac0",dac);
    if (ret < 0) {
      adbg("ERROR:register /dev/dac0 failed(%d)\n",ret);
	    return ret;
    }
    //enable dac channel
    modifyreg32(STM32_DAC_CR, 0, DAC_CR_EN1);
#endif
#ifdef CONFIG_STM32_DAC2
    dac = stm32_dacinitialize(2);
    if (NULL == dac) {
      adbg("ERROR:Failed to get DAC1 interface\n");
	    return -ENODEV;
    }
    ret = dac_register("/dev/dac1",dac);
    if (ret < 0) {
      adbg("ERROR:register /dev/dac0 failed(%d)\n",ret);
	    return ret;
    }
    modifyreg32(STM32_DAC_CR, 0, DAC_CR_EN2);
#endif
    initialized = true;
  }
  
}
#endif
