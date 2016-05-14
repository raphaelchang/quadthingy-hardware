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

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	palSetPadMode(GPIOC, 2, PAL_MODE_INPUT |
			PAL_STM32_OSPEED_HIGHEST);
	palSetPadMode(GPIOC, 1, PAL_MODE_INPUT |
			PAL_STM32_OSPEED_HIGHEST);
	palSetPadMode(GPIOB, 9, PAL_MODE_INPUT |
			PAL_STM32_OSPEED_HIGHEST);
	esc_init();
	chThdSleepMilliseconds(1000);

	comm_usb_serial_init();
	ws2812b_init();
	uint32_t r = 0;
	uint32_t g = 85;
	uint32_t b = 170;
	bno055_init();
	initNRF24L01();
	controller_init();
	for(;;)
	{
		if (comm_usb_serial_is_active())
		{
			uint8_t test[20] = "Hello world";
			chprintf((BaseSequentialStream *)&SDU1, "%s\n", test);
		}
        unsigned long res = 0;
        size_t s = chnReadTimeout(&nrf24l01.channels[1], (uint8_t*)&res, sizeof(unsigned long), MS2ST(100));
        chnWriteTimeout(&nrf24l01.channels[0], (uint8_t*)&res, sizeof(unsigned long), MS2ST(250));
        if (res > 100)
        {
        	res = 0;
        }
        vector3 vector = bno055_get_vector(VECTOR_EULER);
        uint8_t stat = bno055_get_status();
        uint8_t err = bno055_get_error(); 
        controller_update(res / 100.0);
		ws2812b_set_led_color(0, (uint32_t)(128 + 3 * vector.y));
		ws2812b_set_led_color(1, (uint32_t)(128 - 3 * vector.x));
		ws2812b_set_led_color(2, (uint32_t)(128 - 3 * vector.y));
		ws2812b_set_led_color(3, (uint32_t)(128 + 3 * vector.x));
	}
}