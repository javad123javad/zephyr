#include "eeprom_fake.h"
#include "zephyr/drivers/eeprom.h"
#include <sys/types.h>

int e2prom_fake_write(const struct device* fake_e2p_dev, const off_t address, const uint8_t* buf, const size_t buf_len)
{
	int ret = 0;
	
	ret = eeprom_write(fake_e2p_dev, address, buf, buf_len);
	return ret;
}

int e2prom_fake_read(const struct device *fake_e2p_dev, const off_t address, uint8_t *buf, size_t buf_len)
{
	int ret = 0;

	ret = eeprom_read(fake_e2p_dev, address, buf, buf_len);
	
	return ret;
}
