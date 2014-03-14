#ifndef _CF_KEY_H
#define _CF_KEY_H

#define OUTPUT_KEY       (1<<0)
#define UP_KEY           (1<<1)
#define DOWN_KEY         (1<<2)
#define CTRL_KEY         (1<<3)
#define FN_KEY           (1<<4)
#define LEFT_KEY         (1<<5)
#define LEFT_KEY_SHIFT   (5)
#define RIGHT_KEY        (1<<6)
#define RIGHT_KEY_SHIFT  (6)
#define SWITCH_KEY       (1<<7)

int cf_key_init(void);
void cf_key_deinit(void);
mqd_t cf_key_mq_add(char *name);
int cf_key_mq_del(mqd_t mq);
int cf_key_get(void);

#endif
