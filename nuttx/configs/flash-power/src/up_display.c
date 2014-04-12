/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include <sys/types.h>
#include <stdio.h>
#include <errno.h>
#include <debug.h>
#include <stdlib.h>
#include <string.h>
#include <arch/board/board.h>

#include "up_arch.h"
#include "stm32f407-internal.h"
#include "74595lib.h"
#include "font.h"

#define DISP_SIZE      (20)
#define DISP_NUM       (4)
#define DISP_CHANNELS  (2)
#define LED_NUM        (4)
#define INVALID_INDEX  (0xFF)


struct display_element {
  char count;   // display count
  char content[DISP_NUM]; // display content 
};

struct display_buffer {
  struct display_element element[DISP_SIZE];
  char   disp_index;
  char   write_index;
  char   lock_index;
  sem_t  write_sem;
};

struct cf_display {
	struct display_buffer channel[DISP_CHANNELS];
  bool   start;
	//char   led;
};

static struct cf_display g_disp;

void cf_display_init(void)
{
  int i;

  memset(&g_disp, 0, sizeof(g_disp));
  for (i=0; i<DISP_CHANNELS; i++) {
    sem_init(&g_disp.channel[i].write_sem, 0, 1);  
    g_disp.channel[i].lock_index = INVALID_INDEX;
  }
  return;
}

void cf_display_deinit(void)
{
	int i;

	for (i=0; i<DISP_CHANNELS; i++) {
    sem_destroy(&(g_disp.channel[0].write_sem));
    g_disp.channel[i].lock_index = INVALID_INDEX;
  }
  g_disp.start = false;
  return;
}

void cf_display_start(void)
{
  g_disp.start = true;
}


void cf_display_stop(void)
{
  g_disp.start = false;
}
/****************************************************************************
 * Function: cf_display_fresh
 *
 * Description:
 *    send current buffer data to hardware
 ****************************************************************************/
 int cf_display_fresh(void)
 {
  uint8_t data[3];
  char index0,index1;
  int i;
  bool idle_flag0=false,idle_flag1=false,lock_flag0=false,lock_flag1=false;

  if (g_disp.start == false) {
    printf("cf_display_fresh:ERROR:display not started\n");
    return -EPERM;
  }
  if (sem_wait(&g_disp.channel[0].write_sem) != OK) {
    printf("cf_display_fresh:wait for channel 0 sem failed(%d)\n",errno);
    return;
  }
  if (sem_wait(&g_disp.channel[1].write_sem) != OK) {
    printf("cf_display_fresh:wait for channel 1 sem failed(%d)\n",errno);
    return;
  }
  /* init display index*/
  if (INVALID_INDEX != g_disp.channel[0].lock_index) {
    index0 = g_disp.channel[0].lock_index;
    lock_flag0 = true;
  } else {
    index0 = g_disp.channel[0].disp_index;
  }
  
  if (INVALID_INDEX != g_disp.channel[1].lock_index) {
  
    index1 = g_disp.channel[1].lock_index;
    lock_flag1 = true;
  } else {
  
    index1 = g_disp.channel[1].disp_index;
  }
  
  /* check display count, if zero, display idle char('-')*/
  if (g_disp.channel[0].element[index0].count == 0) {
  
    idle_flag0 = true;
  }
  if (g_disp.channel[1].element[index1].count == 0) {
  
    idle_flag1 = true;
  }

  /* display data */
  for (i=0; i<DISP_NUM; i++) {
    if (idle_flag0) {
      data[0] = g_ledfont[FONT_NOP_POS];
    } else {
      data[0] = g_disp.channel[0].element[index0].content[i];
    }
    if (idle_flag1) {
      data[1] = g_ledfont[FONT_NOP_POS];
    } else {
      data[1] = g_disp.channel[1].element[index1].content[i];
    }
  
    data[2] = (1<<i);
    outputbuff_74595(data,3);
    //sleep 1ms
    usleep(1000*1);
    // clear cs
    outputbyte_74595(0);
  }

  /* prepare for next display*/
  if (lock_flag0 == false) {
    if (g_disp.channel[0].element[index0].count == 1) {
      /*this element display finish, switch to next element*/
      g_disp.channel[0].element[index0].count = 0;
      g_disp.channel[0].disp_index = (g_disp.channel[0].disp_index + 1) % DISP_SIZE;
  
    } else if (g_disp.channel[0].element[index0].count > 1){
      g_disp.channel[0].element[index0].count--;
    }
  }
  if (lock_flag1 == false) {
    if (g_disp.channel[1].element[index1].count == 1) {
      /*this element display finish, switch to next element*/
      g_disp.channel[1].element[index1].count = 0;
      g_disp.channel[1].disp_index = (g_disp.channel[1].disp_index + 1) % DISP_SIZE;
  
    } else if (g_disp.channel[1].element[index1].count > 1){
      g_disp.channel[1].element[index1].count--;
    }
  }
  sem_post(&g_disp.channel[0].write_sem);
  sem_post(&g_disp.channel[1].write_sem);
  

}

static char map(char data)
{
  char value = data&(~(1<<7));
  char ret = 0;

  if (value >= 0 && value <= 9) {
    ret = g_ledfont[value];    
  } else if (value >= 0xA && value <= 0xF){
    ret = g_ledfont[FONT_A_POS+value-0xa];
  } else if ( value >='0' && value <= '9') {
    ret = g_ledfont[value-'0'];
  } else if ( value >= 'a' && value <= 'z') {
    ret = g_ledfont[value-'a'+FONT_A_POS];
  } else if ( value >= 'A' && value <= 'Z') {
    ret = g_ledfont[value-'A'+FONT_A_POS];
  } else {
    ret = g_ledfont[FONT_NOP_POS];
  }
  if (data&0x80) {
    ret |= 0x80;
  }
  
  return ret;
}


int cf_display_buffer(int idev, char* buffer, char disp_count)
{
  int i,index,ret;

  if (idev >= DISP_CHANNELS || disp_count == 0) {
    printf("cf_display_buffer:invalid parameter\n");
    return -EINVAL;
  }
  
  if (sem_wait(&g_disp.channel[idev].write_sem) == OK) {
    index = g_disp.channel[idev].write_index;
    ret = index;
    g_disp.channel[idev].element[index].count = disp_count;
    for (i=0; i<DISP_NUM; i++) {
      g_disp.channel[idev].element[index].content[i] = map(buffer[i]);
    }
    /* switch to nex write index*/
    index = (index + 1) % DISP_SIZE;
    g_disp.channel[idev].write_index = index;
    sem_post(&g_disp.channel[idev].write_sem);
    
    return ret;
  } else {
    printf("cf_display_buffer:wait for sem failed(%d)\n",errno);
    return -errno;
  }
}

int cf_display(int idev, int data,char disp_count)
{

  int i,index,ret;

  if ( idev >= DISP_CHANNELS || disp_count == 0) {
    printf("cf_display:ERROR:invaid parameter\n");
    return -EINVAL;
  }

  if (sem_wait(&g_disp.channel[idev].write_sem) == OK) {
    index = g_disp.channel[idev].write_index;
    ret = index;
    g_disp.channel[idev].element[index].count = disp_count;
    for (i=0; i<DISP_NUM; i++) {
      g_disp.channel[idev].element[index].content[i] = map((data>>i*8) & 0xFF);
    }
    /*switch to next write index*/
    index = (index + 1) % DISP_SIZE;
    g_disp.channel[idev].write_index = index;
    sem_post(&g_disp.channel[idev].write_sem);
    return ret;
  } else {
    printf("cf_display:wait for sem failed(%d)\n",errno);
    return -errno;
  }

}

int cf_display_lock(int idev,int index,bool enable) 
{
  if (idev >= DISP_CHANNELS || index >= DISP_SIZE) {
    printf("cf_display_lock:invalid parameter");
    return -EINVAL;
  }
  
  if (sem_wait(&g_disp.channel[idev].write_sem) == OK) {
    if (enable) {
      g_disp.channel[idev].lock_index = index;
    } else {
      g_disp.channel[idev].lock_index = INVALID_INDEX;
    }
    sem_post(&g_disp.channel[idev].write_sem);
    return OK;
  } else {
    printf("cf_display_lock:wait for sem failed(%d)\n",errno);
    return -errno;
  }
}

#if 0
int cf_display_led(int idx, bool enable)
{
  if (idx > LED_NUM) {
    printf("cf_display_led:ERROR:invalid parameter\n");
    return -EINVAL;
  }
  if (sem_wait(&g_disp.display_sem) == OK) {
    g_disp.led &= ~(1<<(idx+4));
    if (enable) {
     g_disp.led |= (1<<(idx+4));
   }
   sem_post(&g_disp.display_sem);
   return OK;
 } else {
  printf("cf_display_led:ERROR:wait sem failed(%d)\n",errno);
  return -errno;
}
}
#endif
