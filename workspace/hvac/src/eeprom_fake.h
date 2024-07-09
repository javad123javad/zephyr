#ifndef __EEPROM_FAKE_H_
#define __EEPROM_FAKE_H_
#include "zephyr/types.h"
#include <stdint.h>
#include <zephyr/device.h>
#include <zephyr/drivers/eeprom.h>

int e2prom_fake_write(const struct device* fake_e2p_dev, const off_t address, const uint8_t* buf, const size_t buf_len);
int e2prom_fake_read(const struct device *fake_e2p_dev, const off_t address, uint8_t *buf, size_t buf_len);


#endif
