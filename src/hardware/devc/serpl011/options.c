/*
 * $QNXLicenseC:
 * Copyright 2007-2019, 2023, BlackBerry Limited.
 *
 * Licensed under the Apache License, Version 2.0 (the "License"). You
 * may not reproduce, modify or distribute this software except in
 * compliance with the License. You may obtain a copy of the License
 * at: http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" basis,
 * WITHOUT WARRANTIES OF ANY KIND, either express or implied.
 *
 * This file may contain contributions from others, either as
 * contributors under the License or as licensors under other terms.
 * Please review this entire file for other proprietary rights or license
 * notices, as well as the QNX Development Suite License Guide at
 * http://licensing.qnx.com/license-guide/ for other information.
 * $
 */

#include "externs.h"

unsigned
options(int argc, char *argv[])
{
	int				opt;
	int				numports = 0;
	void			*link;
	unsigned		rx_fifo = 2;	// 1 / 2 full, default
	unsigned		tx_fifo = 2;	// 1 / 2 full, default
	TTYINIT_PL011	devinit = {
        .tty = {
            .port = 0,
            .port_shift = 1,
            .intr = 0,
            .baud = 38400,
            .isize = 2048,
            .osize = 2048,
            .csize = 256,
            .c_cflag = 0,
            .c_iflag = 0,
            .c_lflag = 0,
            .c_oflag = 0,
            .fifo = 0,
            .clk = 24000000,
            .div = 16,
            .name = "/dev/ser",
            .reserved1 = NULL,
            .reserved2 = 0,
            .verbose = 0,
            .highwater = 0,
            .logging_path = "",
            .lflags = 0,
            .unit = 1
        },
        .drt = 0,
        .prio = 0,
        .loopback = 0,
        .fifo_reg = 0,
        .dma_enable = 0,
        .dma_request_rx = 0,
        .dma_request_tx = 0,
        .dma_xfer_size = 0,
        .chan_rx = 0,
        .chan_tx = 0
	};

	/*
	 * Initialize the devinit to raw mode
	 */
	ttc(TTC_INIT_RAW, &devinit, 0);

	while (optind < argc) {
		/*
		 * Process dash options.
		 */
		while ((opt = getopt(argc, argv, IO_CHAR_SERIAL_OPTIONS "t:T:")) != -1) {
			switch (ttc(TTC_SET_OPTION, &devinit, opt)) {
				case 't':
					rx_fifo = strtoul(optarg, NULL, 0);
					if (rx_fifo > 4)
						rx_fifo = 2;
					break;

				case 'T':
					tx_fifo = strtoul(optarg, NULL, 0);
					if (tx_fifo > 4)
						tx_fifo = 2;

					break;
			}
		}

		devinit.tty.fifo = (rx_fifo << 3) | tx_fifo;

		/*
		 * Process ports and interrupts.
		 */
		while (optind < argc  &&  *(optarg = argv[optind]) != '-') {
			devinit.tty.port = strtoul(optarg, &optarg, 16);
			if (*optarg == '^') {
				devinit.tty.port_shift = strtoul(optarg + 1, &optarg, 0);
			}
			if (*optarg == ',')  {
				devinit.tty.intr = strtoul(optarg + 1, NULL, 0);
			}
			create_device(&devinit);
			devinit.tty.unit++;
			++numports;
			++optind;
		}
	}

	if (numports == 0) {
		link = NULL;
		devinit.tty.fifo = (rx_fifo << 3) | tx_fifo;
		while (1) {
			link = query_default_device(&devinit, link);
			if (link == NULL)
				break;
			create_device(&devinit);
			devinit.tty.unit++;
			++numports;
		}
	}

	return numports;
}
