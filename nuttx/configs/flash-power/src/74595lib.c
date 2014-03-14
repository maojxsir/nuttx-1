 /************************************************************************************
  * configs/chunfeng-powerl/src/74595lib.c
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
 #include <sys/types.h>
 #include <stdio.h>
 #include <errno.h>
 #include <debug.h>
 
 #include <arch/board/board.h>
 
 #include "up_arch.h"
 #include "stm3210e-internal.h"
 
 /************************************************************************************
 * Definitions
 ************************************************************************************/
#define NUM_74595  (4)

static bool g_bout = true;       // ser line default state is output
sem_t g_sem_74595;
/************************************************************************************
 * Private Functions
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/
void init_74595(void)
{
    sem_init(&g_sem_74595,0,1);
}

void deinit_74595(void)
{ 
    sem_destroy(&g_sem_74595);
}

int sendbyte_74595(uint8_t data)
{
    int i;
	if (!g_bout) {
		stm32_configgpio(GPIO_SER_OUT);
		g_bout = true;
	}
	for (i=0; i<8; i++) {
		stm32_gpiowrite(GPIO_SER_OUT,(data<<i)&0x80);
		stm32_gpiowrite(GPIO_SCK, true);
		stm32_gpiowrite(GPIO_SCK, false);
	}
	return 0;
		
}
	
int outputbyte_74595(uint8_t data)
{

	if (sem_wait(&g_sem_74595) == 0 ) {
		sendbyte_74595(data);
		stm32_gpiowrite(GPIO_RCK,1);
		stm32_gpiowrite(GPIO_RCK,0);
#if 0
		sendbyte_74595(0);
		stm32_gpiowrite(GPIO_RCK,1);
		stm32_gpiowrite(GPIO_RCK,0);
#endif
		sem_post(&g_sem_74595);
		return OK;
	} else {
	    printf("outputbyte_74595:wait for sem failed(%d)\n",errno);
		return -errno;
	}
}

int outputbuff_74595(uint8_t *buf, int size)
{
  int i;
	
  if (NULL == buf || size > NUM_74595) {
		printf("outputbuff_74595:ERROR,invalid parameter\n");
		return -EINVAL;
  }
  if (sem_wait(&g_sem_74595) == OK) {
    for (i=0; i<size; i++) {
	   sendbyte_74595(buf[i]);
	  }
	  //output enable
	  stm32_gpiowrite(GPIO_RCK,1);
	  stm32_gpiowrite(GPIO_RCK,0);
	  sem_post(&g_sem_74595);
	  return 0;
  } else {
    printf("outputbuff_74595:wait for sem failed(%d)\n",errno);
	  return -errno;
  }
}

int readbyte_74595(uint32_t *key)
{
    int i,ret=0;
#if 0
	if (buff && size != 3) {
		printf("readbyte_74595:ERROR invalid paremeter\n");
		return -EINVAL;
	}
#endif

    if (sem_wait(&g_sem_74595) == OK) {
        for (i=0; i<8; i++) {
            sendbyte_74595(1<<i);
            sendbyte_74595(0);
            sendbyte_74595(0);
            sendbyte_74595(0);
            //output enable
            stm32_gpiowrite(GPIO_RCK,1);
            stm32_gpiowrite(GPIO_RCK,0);
            //read GPIO_SER
            stm32_configgpio(GPIO_SER_IN);
            g_bout = false;
            if (stm32_gpioread(GPIO_SER_IN)) {
                ret |= 1<<i;
            }
        }
        *key = ret;
        sem_post(&g_sem_74595);
        return OK;
    } else {
        printf("readbyte_74595:wait for sem failed(%d)\n",errno);
        return -errno;
    }
}