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
#include <mqueue.h>
#include <fcntl.h>
#include <nuttx/kmalloc.h>
#include <arch/board/board.h>

#include "up_arch.h"
#include "stm3210e-internal.h"
#include "74595lib.h"
#include "up_key.h"

struct mq_link {
    struct mq_link *next;
    struct mq_link *pre;
    mqd_t value;
};

#define MQ_WRITE_PRIO  (1)

#if 0
#define STATE_00   (0)
#define STATE_01   (1)
#define STATE_10   (2)
#define STATE_11   (3)
#endif

#define DEBOUNCE_CNT   (7)
#define LONG_PRESS_CNT (20)
#define KEY_CNT        (6)   /* bit6:bit7 for rotary encoder, not support now*/


static struct mq_link g_mq_list = {0};
static sem_t          g_cf_key_sem;

int cf_key_init(void)
{
    sem_init(&g_cf_key_sem,0,1);
    return OK;
}

void cf_key_deinit(void)
{
    sem_destroy(&g_cf_key_sem);
}

mqd_t cf_key_mq_add(char *name)
{
    mqd_t mq = NULL;
    struct mq_link *link = NULL;

    if (name == NULL) {
        printf("cf_key_mq_add:ERROR:invalid parameter\n");
        set_errno(EINVAL);
        return NULL;
    }
    mq = mq_open(name,O_WRONLY|O_NONBLOCK);
    if (mq < 0) {
        printf("cf_key_mq_add:ERROR:mq_open failed(%d)\n",errno);
        return mq;
    }
    link = kzalloc(sizeof(*link));
    if (link == NULL) {
        printf("cf_key_mq_add:ERROR:kzalloc failed\n");
        goto error;
    }
    link->value = mq;

    if (sem_wait(&g_cf_key_sem) == OK) {
        link->next = g_mq_list.next;
        link->pre  = &g_mq_list;
        if (g_mq_list.next) {
          g_mq_list.next->pre = link;
        }
        g_mq_list.next = link;
        sem_post(&g_cf_key_sem);
    } else {
        printf("cf_key_mq_add:ERROR:wait for sem failed(%d)\n",errno);
        goto error;
    }

    return mq;

error:
    if (mq >= 0 ){
        mq_close(mq);
    }
    if (link) {
        kfree(link);
    }
    return (mqd_t)ERROR;
}

int cf_key_mq_del(mqd_t mq)
{
    struct mq_link *link = g_mq_list.next;

	while(link != NULL) {
        if (link->value == mq) {
            break;
		}
		link = link->next;
	}
	if (NULL == link) {
        printf("cf_key_mq_del:no find mq\n");
		return -ENXIO;
	}
	if (sem_wait(&g_cf_key_sem) == OK) {
    	mq_close(link->value);

	    link->pre->next = link->next;
	    link->next->pre = link->pre;

		kfree(link);
		sem_post(&g_cf_key_sem);
	} else {
        printf("cf_key_mq_del:ERROR:wait for sem failed(%d)\n",errno);
		return -EAGAIN;
	}

	return OK;
}

uint8_t cf_key_map(uint8_t rawkey) 
{
  static uint8_t previous_key = 0;
  static uint32_t count[KEY_CNT] = {0};
  int i;
  uint8_t ret_key = 0;
  uint8_t xor_result = 0;
  
  /*Debounce logic*/
  xor_result = rawkey ^ previous_key;
  if (xor_result) {
    // xor not zero,check every bit in xor result to know bit change status
    for (i=0; i<KEY_CNT; i++) {
      if (xor_result & (1<<i)) {
        // bit is 1, status change
        if (rawkey & (1<<i)) {
          // now status is 1,previous status is zero. 0 -> 1
          count[i] = 1;
        } else {
          // now status is zero, previos status is 1. 1 -> 0
          count[i] = 0;
        }
      } else {
        // bit is zero, status not change
        if (rawkey & (1<<i)) {
          // now status and previous status both 1. 1 -> 1
          count[i] ++;
        }
      }
    }
  } else {
    // xor zero.
    if (previous_key == 0) {
      // previous is zero. previous and rawkey is same zero. nothing to do ,just return zero;
      goto _nocheck_ret;
    } else {
      // previous not zero, check every bit in rawkey
      for (i=0; i<KEY_CNT; i++) {
        if (rawkey & (1<<i)) {
          // bit is 1 count++
          count[i] ++;
        }
      }
    }
  }

  /*check count if large than DEBOUNCE Count,report key*/
  for (i=0; i<KEY_CNT; i++) {
    if ( count[i] == DEBOUNCE_CNT || (count[i] >= (DEBOUNCE_CNT+LONG_PRESS_CNT)) ) {
      ret_key |= 1<<i;
    } 
  }


_nocheck_ret:
  previous_key = rawkey;
  return ret_key;
}

int cf_key_get(void)
{
    uint8_t   rawkey;
    uint8_t   key;
    struct mq_link   *link = NULL;
    
	if (readbyte_74595(&rawkey) != OK) {
        printf("cf_key_get:ERROR:failed to get key from 74595\n");
	    return -EAGAIN;
	} else {
    key = cf_key_map(rawkey);
    
    if (key == 0 ) {
      return 0;
    }
    
	  if (sem_wait(&g_cf_key_sem) == OK) {
    
	    link = g_mq_list.next;
	    while (link != NULL) {
	      if (mq_send(link->value, &key, 1,MQ_WRITE_PRIO) != OK) {
	        printf("cf_key_get:ERROR:failed to write key into mq(%d)\n",errno);
	      }
	      link = link->next;
	    }
	    sem_post(&g_cf_key_sem);
	  } else {
      printf("cf_key_get:ERROR:wait for sem failed(%d)\n",errno);
	  	return -EAGAIN;
	  }
	}
  return OK;
}

