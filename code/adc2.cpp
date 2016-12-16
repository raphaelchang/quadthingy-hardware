#include "adc2.h"
#include "ch.h"
#include "hal.h"

#define ADC_GRP1_NUM_CHANNELS   1
#define ADC_GRP1_BUF_DEPTH      8

static adcsample_t samples[ADC_GRP1_NUM_CHANNELS * ADC_GRP1_BUF_DEPTH];

static void adcerrorcallback(ADCDriver *adcp, adcerror_t err) {

    (void)adcp;
    (void)err;
}

/*
 *  * ADC conversion group.
 *   * Mode:        Linear buffer, 8 samples of 1 channel, SW triggered.
 *    * Channels:    IN11.
 *     */
static const ADCConversionGroup adcgrpcfg1 = {
    FALSE,
    ADC_GRP1_NUM_CHANNELS,
    NULL,
    adcerrorcallback,
    0,                        /* CR1 */
    ADC_CR2_SWSTART,          /* CR2 */
    ADC_SMPR1_SMP_AN11(ADC_SAMPLE_3),
    0,                        /* SMPR2 */
    ADC_SQR1_NUM_CH(ADC_GRP1_NUM_CHANNELS),
    0,                        /* SQR2 */
    ADC_SQR3_SQ1_N(ADC_CHANNEL_IN10)
};

void adc_init(void)
{
    palSetPadMode(GPIOC, 0, PAL_MODE_INPUT_ANALOG);
    adcStart(&ADCD1, NULL);
    adcSTM32EnableTSVREFE();
}

float adc_get_sys_voltage(void)
{
    adcConvert(&ADCD1, &adcgrpcfg1, samples, ADC_GRP1_BUF_DEPTH);
    int i = 0;
    float avg = 0;
    for (i = 0; i < ADC_GRP1_BUF_DEPTH; i++)
    {
        float v = samples[i] * 3.3 / 4096 * (15.1 / 10);
        avg += v;
    }
    avg /= ADC_GRP1_BUF_DEPTH;
    return avg;
}
