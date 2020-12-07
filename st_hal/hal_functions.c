#include "ch.h"
#include "hal.h"

uint32_t HAL_GetTick(void) {
	return TIME_I2MS(chVTGetSystemTimeX());
}
