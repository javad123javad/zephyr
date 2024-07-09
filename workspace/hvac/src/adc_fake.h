#ifndef _ADC_FAKE_H_
#define _ADC_FAKE_H_
#include <stdint.h>
#include <zephyr/device.h>
typedef struct _FAKE_ADC_DEV
{
	const struct device * adc_dev;
	uint16_t input_mv;
	uint16_t nsamples;
}fake_adc_dev_t;

int adc_fake_setup(const fake_adc_dev_t* fake_adc_dev);
int adc_fake_read(const fake_adc_dev_t* fake_adc_dev);
int adc_fake_set_value(const fake_adc_dev_t * fake_adc_dev);
#endif
