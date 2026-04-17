/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "zephyr/sys/printk.h"
#include <stdint.h>
#include <stdio.h>
#include <zephyr/net_buf.h>
NET_BUF_SIMPLE_DEFINE(demo_buff, 64);

int main(void)
{
        net_buf_simple_init(&demo_buff, 0);  // reserve 1 byte headroom

        const uint8_t tx_buf[16] = "HELLO\r\n";
        uint8_t rx_buf[16] = {0};

        net_buf_simple_add_mem(&demo_buff, tx_buf, sizeof(tx_buf));
#if 0
        for(int i = sizeof(buf)-1; i >=0; i--)
        {
                printk("%c", buf[i]);
                net_buf_simple_add_u8(&demo_buff, buf[i]);
        }
        for(int i =0; i <net_buf_simple_max_len(&demo_buff); i++)
        {
                uint8_t ch = net_buf_simple_remove_u8(&demo_buff);
                printk("%d\n", ch);
                rxBuf[i] = ch;
        }

        printk("Rx Buf: %s\n", rxBuf);
#endif
        printk("rx: %s\n",net_buf_simple_remove_mem(&demo_buff, sizeof(tx_buf)));


	return 0;
}
