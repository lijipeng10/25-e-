#include "buzzer.h"

void buzzer_on(void)
{
    DL_GPIO_setPins(buzzer_PORT, buzzer_BUZZER_PIN);
}

void buzzer_off(void)
{
    DL_GPIO_clearPins(buzzer_PORT, buzzer_BUZZER_PIN);
}

