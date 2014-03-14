#ifndef _74595LIB_H
#define _74595LIB_H

void init_74595(void);
void deinit_74595(void);
int sendbyte_74595(uint8_t data);
int outputbyte_74595(uint8_t data);
int outputbuff_74595(uint8_t *buf, int size);
int readbyte_74595(uint32_t *key);

#endif
