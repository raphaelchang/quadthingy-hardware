#include "ch.h"
#include "hal.h"
#include "stm32f4xx_conf.h"
#include "comm_usb.h"
#include "ws2812b.h"
#include "bno055.h"
#include "esc.h"
#include "nrf24l01.h"
#include "controller.h"
#include "chprintf.h"
#include <stdio.h>
#include <string.h>
#include "math.h"

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
	esc_init();
	initNRF24L01();
	for(;;)
	{
		if (comm_usb_serial_is_active())
		{
			uint8_t test[20] = "Hello world";
			chprintf((BaseSequentialStream *)&SDU1, "%s\n", test);
		}
		//uint8_t serialInBuf[32];
        //chnWriteTimeout(&nrf24l01.channels[0], serialInBuf, 32, MS2ST(1000));
        vector3 vector = bno055_get_vector(VECTOR_EULER);
        uint8_t stat = bno055_get_status();
        uint8_t err = bno055_get_error(); 
        esc_set_all(0.0);
        controller_update();
		ws2812b_set_led_color(0, (uint32_t)(128 + 3 * vector.y));
		ws2812b_set_led_color(1, (uint32_t)(128 - 3 * vector.x));
		ws2812b_set_led_color(2, (uint32_t)(128 - 3 * vector.y));
		ws2812b_set_led_color(3, (uint32_t)(128 + 3 * vector.x));
		chThdSleepMilliseconds(10);
	}
}