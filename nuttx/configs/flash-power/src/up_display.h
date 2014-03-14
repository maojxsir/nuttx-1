#ifndef __CF_UP_DISPLAY_H
#define __CF_UP_DISPLAY_H

void cf_display_init(void);
void cf_display_deinit(void);
int cf_display_start(void);
int cf_display_stop(void);
int cf_display_fresh(void);
int cf_display(int idev,int pos, char data);
int cf_display4(int idev, int data);
int cf_display_selffresh(int idev,bool enable);
int cf_display_led(int idx, bool enable);

#endif