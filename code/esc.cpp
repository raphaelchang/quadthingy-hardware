#include "ch.h"
#include "hal.h"
#include "stm32f4xx_conf.h"
#include "esc.h"

#define ESC_UPDATE_RATE		10000
#define TIM_CLOCK			10000000
#define ALL_CHANNELS		0xFF

static PWMConfig pwmcfg = {
		TIM_CLOCK,
		(uint16_t)((uint32_t)TIM_CLOCK / (uint32_t)ESC_UPDATE_RATE),
		NULL,
		{
				{PWM_OUTPUT_ACTIVE_HIGH, NULL},
				{PWM_OUTPUT_ACTIVE_HIGH, NULL},
				{PWM_OUTPUT_ACTIVE_HIGH, NULL},
				{PWM_OUTPUT_ACTIVE_HIGH, NULL}
		},
		0,
		0
};

void esc_init(void) {
	pwmStart(&PWMD5, &pwmcfg);

	palSetPadMode(GPIOA, 0,
			PAL_MODE_ALTERNATE(GPIO_AF_TIM5) |
			PAL_STM32_OTYPE_PUSHPULL |
			PAL_STM32_OSPEED_MID1);
	palSetPadMode(GPIOA, 1,
			PAL_MODE_ALTERNATE(GPIO_AF_TIM5) |
			PAL_STM32_OTYPE_PUSHPULL |
			PAL_STM32_OSPEED_MID1);
	palSetPadMode(GPIOA, 2,
			PAL_MODE_ALTERNATE(GPIO_AF_TIM5) |
			PAL_STM32_OTYPE_PUSHPULL |
			PAL_STM32_OSPEED_MID1);
	palSetPadMode(GPIOA, 3,
			PAL_MODE_ALTERNATE(GPIO_AF_TIM5) |
			PAL_STM32_OTYPE_PUSHPULL |
			PAL_STM32_OSPEED_MID1);
	palSetPadMode(GPIOC, 6, PAL_MODE_OUTPUT_PUSHPULL |
			PAL_STM32_OTYPE_PUSHPULL |
			PAL_STM32_OSPEED_HIGHEST);
	palSetPadMode(GPIOC, 7, PAL_MODE_OUTPUT_PUSHPULL |
			PAL_STM32_OTYPE_PUSHPULL |
			PAL_STM32_OSPEED_HIGHEST);
	palSetPadMode(GPIOC, 8, PAL_MODE_OUTPUT_PUSHPULL |
			PAL_STM32_OTYPE_PUSHPULL |
			PAL_STM32_OSPEED_HIGHEST);
	palSetPadMode(GPIOC, 9, PAL_MODE_OUTPUT_PUSHPULL |
			PAL_STM32_OTYPE_PUSHPULL |
			PAL_STM32_OSPEED_HIGHEST);
	palSetPad(GPIOC, 6);
	palSetPad(GPIOC, 7);
	palSetPad(GPIOC, 8);
	palSetPad(GPIOC, 9);

	esc_set_all(0);
}

void esc_set(uint8_t channel, double duty_cycle) {
	uint32_t cnt_val;
	if (duty_cycle < 0)
		duty_cycle = 0;
	if (duty_cycle > 1)
		duty_cycle = 1;

	cnt_val = (uint32_t)(duty_cycle * (uint32_t)TIM_CLOCK / (uint32_t)ESC_UPDATE_RATE);

	switch(channel) {
	case 0:
		pwmEnableChannel(&PWMD5, 0, cnt_val);
		break;

	case 1:
		pwmEnableChannel(&PWMD5, 1, cnt_val);
		break;

	case 2:
		pwmEnableChannel(&PWMD5, 2, cnt_val);
		break;

	case 3:
		pwmEnableChannel(&PWMD5, 3, cnt_val);
		break;

	case ALL_CHANNELS:
		pwmEnableChannel(&PWMD5, 0, cnt_val);
		pwmEnableChannel(&PWMD5, 1, cnt_val);
		pwmEnableChannel(&PWMD5, 2, cnt_val);
		pwmEnableChannel(&PWMD5, 3, cnt_val);
		break;

	default:
		break;
	}
}

void esc_set_all(double duty_cycle) {
	esc_set(ALL_CHANNELS, duty_cycle);
}