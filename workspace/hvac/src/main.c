/*
 * Copyright (c) 2020 Libre Solar Technologies GmbH
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <inttypes.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>
// Ethernet
#include <zephyr/net/socket.h>
#include <zephyr/net/ethernet.h>
#include <zephyr/net/net_mgmt.h>

#include "adc_fake.h"
#include "eeprom_fake.h"
#include "zephyr/net/net_ip.h"
#include "fixedpoint-pid.h"
#define ADC_DEVICE_NODE		DT_INST(0, zephyr_adc_emul)
static const struct device *eeprom;
#define BIND_PORT 4242

int eth_init()
{
	int opt;
	socklen_t optlen = sizeof(int);
	int serv, ret;
	struct sockaddr_in bind_addr = {

		.sin_family = AF_INET,
		.sin_addr = INADDR_ANY_INIT,
		.sin_port = htons(BIND_PORT),
	};
	static int counter;

	serv = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
	if (serv < 0) {
		printf("error: socket: %d\n", errno);
		exit(1);
	}

	ret = getsockopt(serv, IPPROTO_IP, 0, &opt, &optlen);
	if (ret == 0) {
		if (opt) {
			printf("IPV6_V6ONLY option is on, turning it off.\n");

			opt = 0;
			ret = setsockopt(serv, IPPROTO_IP, 0,
					&opt, optlen);
			if (ret < 0) {
				printf("Cannot turn off IPV6_V6ONLY option\n");
			} else {
				printf("Sharing same socket between IPv6 and IPv4\n");
			}
		}
	}

	if (bind(serv, (struct sockaddr *)&bind_addr, sizeof(bind_addr)) < 0) {
		printf("error: bind: %d\n", errno);
		exit(1);
	}

	if (listen(serv, 5) < 0) {
		printf("error: listen: %d\n", errno);
		exit(1);
	}
	printf("Eth init finished\n");
	printf("Single-threaded TCP echo server waits for a connection on "
			"port %d...\n", BIND_PORT);

	while (1) {
		struct sockaddr_in client_addr;
		socklen_t client_addr_len = sizeof(client_addr);
		char addr_str[32];
		int client = accept(serv, (struct sockaddr *)&client_addr,
				&client_addr_len);

		if (client < 0) {
			printf("error: accept: %d\n", errno);
			continue;
		}

		inet_ntop(client_addr.sin_family, &client_addr.sin_addr,
				addr_str, sizeof(addr_str));
		printf("Connection #%d from %s\n", counter++, addr_str);

		while (1) {
			char buf[128], *p;
			int len = recv(client, buf, sizeof(buf), 0);
			int out_len;

			if (len <= 0) {
				if (len < 0) {
					printf("error: recv: %d\n", errno);
				}
				break;
			}

			p = buf;
			do {
				out_len = send(client, p, len, 0);
				if (out_len < 0) {
					printf("error: send: %d\n", errno);
					goto error;
				}
				p += out_len;
				len -= out_len;
			} while (len);
		}

error:
		close(client);
		printf("Connection from %s closed\n", addr_str);
	}

	return 0;
}
int init_pid_controller(FixedPid * pid) {
	FixedPid_Init(pid);
	pid->Dt = Fixed32_FromFloat(0.1);
	pid->Max = 100;// Precent %
	pid->Min = -100;// Precent %
	pid->Kp = Fixed32_FromFloat(0.1);
	pid->Kd = Fixed32_FromFloat(0.01);
	pid->Ki = Fixed32_FromFloat(0.5);

	printf("max=%d    min=%d    dt=%d\n", pid->Max, pid->Min, pid->Dt);

	return 0;
}

int main()
{
	int ret = 0;
	uint8_t tbuf[] ={0xAA, 0xBB};
	const struct device *const adc_dev = DEVICE_DT_GET(ADC_DEVICE_NODE);
	eeprom = DEVICE_DT_GET(DT_ALIAS(eeprom_1));
	fake_adc_dev_t fake_adc={

		.adc_dev = adc_dev,
		.input_mv=1500,
		.nsamples=5,
	};
	/******** EEPROM INIT *******/
	size_t size;
	size = eeprom_get_size(eeprom);
	printk("EEPROM Size: %d\n", size);
	ret = e2prom_fake_write(eeprom, 0x00, tbuf, sizeof(tbuf));
	printk("EEPROM WRITE ret: %d\n",ret);
	uint8_t rbuf[2] = {0};
	ret = e2prom_fake_read(eeprom, 0x00, rbuf, 2);
	printk("ret: %d, buf: %02x, %02x\n", ret, rbuf[0], rbuf[1]);
	/****** PID Controller init ********/ 
	FixedPid pid;
	//init_pid_controller(&pid);
	FixedPid_Init(&pid);
	pid.Dt = Fixed32_FromFloat(0.2);
	pid.Max = 16384;// Precent %
	pid.Min = -16384;// Precent %
	pid.Kp = Fixed32_FromFloat(0.1);
	pid.Kd = Fixed32_FromFloat(0.01);
	pid.Ki = Fixed32_FromFloat(0.5);

	int ftemp = 2500;// Desired temperature	

	/*********ADC INIT **************************/
	ret = adc_fake_setup(&fake_adc);
	printk("fake adc ret: %d\n", ret);
	/******** Main Loop ************************/
	while (1) {
		int ctemp = adc_fake_read(&fake_adc);
		printk("ADC Read val: %d\n", ctemp);

		Fixed32 inc = FixedPid_Calculate(&pid, ftemp, ctemp);
		printk("Current Temp: %d, Desired Temp: %d, motor bias: %d\n", ctemp, ftemp, inc);

		k_msleep(10);
		fake_adc.input_mv =ctemp + inc;
		adc_fake_set_value(&fake_adc);
	}

#if 0U	

	eth_init();
#endif
	return 0;
}
