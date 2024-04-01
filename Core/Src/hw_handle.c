#include "hw_handle.h"

uint16_t hl_pins[2] = {HL1_Pin, HL2_Pin};


int32_t ToggleLed(int32_t n) {
	if(n < 2 && n >= 0)
		HAL_GPIO_TogglePin(GPIOE, hl_pins[n]);
	else
		HAL_GPIO_TogglePin(GPIOE, hl_pins[1]);
	return 0;
}
