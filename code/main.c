#include "ch.h"
#include "hal.h"
#include "stm32f4xx_conf.h"
#include "comm_usb.h"
#include "ws2812b.h"
#include "bno055.h"
#include "chprintf.h"
#include <stdio.h>
#include <string.h>

int main(void) {
	halInit();
	chSysInit();

	chThdSleepMilliseconds(1000);

	comm_usb_serial_init();


	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	palSetPadMode(GPIOC, 2, PAL_MODE_INPUT |
			PAL_STM32_OSPEED_HIGHEST);
	palSetPadMode(GPIOB, 9, PAL_MODE_INPUT |
			PAL_STM32_OSPEED_HIGHEST);
	ws2812b_init();
	uint32_t r = 0;
	uint32_t g = 85;
	uint32_t b = 170;
	bno055_init();
	for(;;)
	{
		if (comm_usb_serial_is_active())
		{
			uint8_t test[20] = "Hello world";
			chprintf((BaseSequentialStream *)&SDU1, "%s\n", test);
		}
		ws2812b_set_led_color(0, ((r % 256) << 16) | ((g % 256) << 8) | ((b % 256)));
		ws2812b_set_led_color(1, (((r + 64) % 256) << 16) | (((g + 64) % 256) << 8) | (((b + 64) % 256)));
		ws2812b_set_led_color(2, (((r + 128) % 256) << 16) | (((g + 128) % 256) << 8) | (((b + 128) % 256)));
		ws2812b_set_led_color(3, (((r + 192) % 256) << 16) | (((g + 192) % 256) << 8) | (((b + 192) % 256)));
		r += 4;
		g += 4;
		b += 4;
		chThdSleepMilliseconds(10);
	}
}