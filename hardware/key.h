#ifndef KEY_H
#define KEY_H

/*
    KEY1 <==> PA29
    KEY2 <==> PB27
    (KEY3 已移除: PB26 让给 OLED 的 BLK 背光脚)
*/


#include "ti_msp_dl_config.h"

void key_init(void);
uint8_t key_getnum(void);
uint8_t key_get_state(void);
void key_tick(void);


#endif // KEY_H
