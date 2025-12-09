/*
 * $QNXLicenseC:
 * Copyright 2019 NXP
 * Copyright 2020, QNX Software Systems.
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

#ifndef IMX_IOMUXC_GPR_H_
#define IMX_IOMUXC_GPR_H_

/* GPC registers, offset from base address */
#define IMX_IOMUXC_GPR0                             0x00     /* General Purpose Register 0 */
#define IMX_IOMUXC_GPR1                             0x04     /* General Purpose Register 1 */
#define IMX_IOMUXC_GPR2                             0x08     /* General Purpose Register 2 */
#define IMX_IOMUXC_GPR3                             0x0C     /* General Purpose Register 3 */
#define IMX_IOMUXC_GPR4                             0x10     /* General Purpose Register 4 */
#define IMX_IOMUXC_GPR5                             0x14     /* General Purpose Register 5 */
#define IMX_IOMUXC_GPR6                             0x18     /* General Purpose Register 6 */
#define IMX_IOMUXC_GPR7                             0x1C     /* General Purpose Register 7 */
#define IMX_IOMUXC_GPR8                             0x20     /* General Purpose Register 8 */
#define IMX_IOMUXC_GPR9                             0x24     /* General Purpose Register 9 */
#define IMX_IOMUXC_GPR10                            0x28     /* General Purpose Register 10 */
#define IMX_IOMUXC_GPR11                            0x2C     /* General Purpose Register 11 */
#define IMX_IOMUXC_GPR12                            0x30     /* General Purpose Register 12 */
#define IMX_IOMUXC_GPR13                            0x34     /* General Purpose Register 13 */
#define IMX_IOMUXC_GPR14                            0x38     /* General Purpose Register 14 */
#define IMX_IOMUXC_GPR15                            0x3C     /* General Purpose Register 15 */
#define IMX_IOMUXC_GPR16                            0x40     /* General Purpose Register 16 */
#define IMX_IOMUXC_GPR17                            0x44     /* General Purpose Register 17 */
#define IMX_IOMUXC_GPR18                            0x48     /* General Purpose Register 18 */
#define IMX_IOMUXC_GPR19                            0x4C     /* General Purpose Register 19 */
#define IMX_IOMUXC_GPR20                            0x50     /* General Purpose Register 20 */
#define IMX_IOMUXC_GPR21                            0x54     /* General Purpose Register 21 */
#define IMX_IOMUXC_GPR22                            0x58     /* General Purpose Register 22 */
#define IMX_IOMUXC_GPR(n)                           (0x5C + (4 * n)) /* General Purpose Register 0 - 24 */

#endif /* IMX_DDRC_H_ */
