/*
 * Copyright (c) 2024 Javad Rahimipetroudi <javad.rahimipetroud@mind.be>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "adc_fake.h"
#include "zephyr/sys/__assert.h"
#include <inttypes.h>
#include <stddef.h>
#include <stdint.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>
#include <zephyr/drivers/adc/adc_emul.h>
//#include <zephyr/ztest.h>
/* Constants */ 
#define ADC_REF_INTERNAL_MV	DT_PROP(DT_INST(0, zephyr_adc_emul), ref_internal_mv)
#define ADC_REF_EXTERNAL1_MV	DT_PROP(DT_INST(0, zephyr_adc_emul), ref_external1_mv)


#define ADC_RESOLUTION		14
#define ADC_ACQUISITION_TIME	ADC_ACQ_TIME_DEFAULT
#define ADC_1ST_CHANNEL_ID	0
#define ADC_2ND_CHANNEL_ID	1

#define INVALID_ADC_VALUE	SHRT_MIN
/* Raw to millivolt conversion doesn't handle rounding */
#define MV_OUTPUT_EPS		2
#define SEQUENCE_STEP		100

#define BUFFER_SIZE  6
static int16_t m_sample_buffer[BUFFER_SIZE];


/**
 * @brief Setup channel with specific reference and gain
 *
 * @param adc_dev Pointer to ADC device
 * @param ref ADC reference voltage source
 * @param gain Gain applied to ADC @p channel
 * @param channel ADC channel which is being setup
 *
 * @return none
 */
int channel_setup(const struct device *adc_dev, enum adc_reference ref,
		enum adc_gain gain, int channel)
{
	int ret;
	struct adc_channel_cfg channel_cfg = {
		.gain             = gain,
		.reference        = ref,
		.acquisition_time = ADC_ACQUISITION_TIME,
		.channel_id       = channel,
	};

	ret = adc_channel_setup(adc_dev, &channel_cfg);
	//	zassert_ok(ret, "Setting up of the %d channel failed with code %d",
	//		   channel, ret);

	return ret;
}

/**
 * @brief Run adc_read for given channels and collect specified number of
 *        samples.
 *
 * @param adc_dev Pointer to ADC device
 * @param channel_mask Mask of channels that will be sampled
 * @param samples Number of requested samples for each channel
 *
 * @return none
 */
static  int start_adc_read(const struct device *adc_dev, uint32_t channel_mask,
	uint16_t len)
{
	int ret;
	const struct adc_sequence_options *options_ptr;

	const struct adc_sequence_options options = {
		.extra_samplings = len - 1,
	};

	if (len > 1) {
		options_ptr = &options;
	} else {
		options_ptr = NULL;
	}

	const struct adc_sequence sequence = {
		.options     = options_ptr,
		.channels    = channel_mask,
		.buffer      = m_sample_buffer,
		.buffer_size = sizeof(m_sample_buffer),
		.resolution  = ADC_RESOLUTION,
	};

	ret = adc_read(adc_dev, &sequence);
	printk("adc read ret: %d\n", ret);
	//	zassert_ok(ret, "adc_read() failed with code %d", ret);
	return ret;
}

int adc_fake_setup(const fake_adc_dev_t* fake_adc_dev)
{
	int ret, i;

	for (i = 0; i < BUFFER_SIZE; ++i) {
		m_sample_buffer[i] = INVALID_ADC_VALUE;
	}

	/* Generic ADC setup */

	ret = channel_setup(fake_adc_dev->adc_dev, ADC_REF_INTERNAL, ADC_GAIN_1,
			ADC_1ST_CHANNEL_ID);
	if(ret)
		return ret;
	/* ADC emulator-specific setup */
	return  adc_emul_const_value_set(fake_adc_dev->adc_dev, ADC_1ST_CHANNEL_ID,fake_adc_dev->input_mv);
}

int adc_fake_read(const fake_adc_dev_t * fake_adc_dev)
{
	int ret;
	int avg_val = 0;
	/* Test sampling */
	ret = start_adc_read(fake_adc_dev->adc_dev, BIT(ADC_1ST_CHANNEL_ID), fake_adc_dev->nsamples);
	
	printk("ADC READ RET :%d\n", ret);
	if(ret)
		return ret;
	
	for(int i = 0; i < fake_adc_dev->nsamples; i++)
	{
		int output = m_sample_buffer[i];
		ret = adc_raw_to_millivolts(ADC_REF_INTERNAL_MV, ADC_GAIN_1, ADC_RESOLUTION,&output);
		avg_val += output;
	}

	return (avg_val/fake_adc_dev->nsamples);
}

int adc_fake_set_value(const fake_adc_dev_t * fake_adc_dev)
{
	return adc_emul_const_value_set(fake_adc_dev->adc_dev, ADC_1ST_CHANNEL_ID, fake_adc_dev->input_mv);
}
