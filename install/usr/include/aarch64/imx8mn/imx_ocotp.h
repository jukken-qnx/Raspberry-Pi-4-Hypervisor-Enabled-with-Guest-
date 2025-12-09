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

#ifndef IMX_OCOTP_H_
#define IMX_OCOTP_H_

/* OCOTP Registers, offset from base address */
#define IMX_HW_OCOTP_CTRL(n)                    (0x00 + (4 * n) /* OTP Controller Control Register */
#define IMX_HW_OCOTP_TIMING                     0x010       /* OTP Controller Timing Register */
#define IMX_HW_OCOTP_DATA                       0x020       /* OTP Controller Write Data Register */
#define IMX_HW_OCOTP_READ_CTRL                  0x030       /* OTP Controller Write Data Register */
#define IMX_HW_OCOTP_READ_FUSE_DATA             0x040       /* OTP Controller Read Data Register */
#define IMX_HW_OCOTP_SW_STICKY                  0x050       /* Sticky bit Register */
#define IMX_HW_OCOTP_SCS(n)                     (0x060 + (4 * n)) /* Software Controllable Signals Register */
#define IMX_HW_OCOTP_VERSION                    0x090       /* OTP Controller Version Register */
#define IMX_HW_OCOTP_LOCK                       0x400       /* Value of OTP Bank0 Word0 */
#define IMX_HW_OCOTP_TESTER0                    0x410       /* Value of OTP Bank0 Word1 */
#define IMX_HW_OCOTP_TESTER1                    0x420       /* Value of OTP Bank0 Word2 */
#define IMX_HW_OCOTP_TESTER2                    0x430       /* Value of OTP Bank0 Word3 */
#define IMX_HW_OCOTP_TESTER3                    0x440       /* Value of OTP Bank1 Word0 */
#define IMX_HW_OCOTP_TESTER4                    0x450       /* Value of OTP Bank1 Word1 */
#define IMX_HW_OCOTP_TESTER5                    0x460       /* Value of OTP Bank1 Word2 */
#define IMX_HW_OCOTP_BOOT_CFG0                  0x470       /* Value of OTP Bank1 Word3 */
#define IMX_HW_OCOTP_BOOT_CFG1                  0x480       /* Value of OTP Bank2 Word0 */
#define IMX_HW_OCOTP_BOOT_CFG2                  0x490       /* Value of OTP Bank2 Word1 */
#define IMX_HW_OCOTP_BOOT_CFG3                  0x4A0       /* Value of OTP Bank2 Word2 */
#define IMX_HW_OCOTP_BOOT_CFG4                  0x4B0       /* Value of OTP Bank2 Word3 */
#define IMX_HW_OCOTP_MEM_TRIM0                  0x4C0       /* Value of OTP Bank3 Word0 */
#define IMX_HW_OCOTP_MEM_TRIM1                  0x4D0       /* Value of OTP Bank3 Word1 */
#define IMX_HW_OCOTP_ANA0                       0x4E0       /* Value of OTP Bank3 Word2 */
#define IMX_HW_OCOTP_ANA1                       0x4F0       /* Value of OTP Bank3 Word3 */
#define IMX_HW_OCOTP_SRK0                       0x580       /* Shadow Register for OTP Bank6 Word0 */
#define IMX_HW_OCOTP_SRK1                       0x590       /* Shadow Register for OTP Bank6 Word1 */
#define IMX_HW_OCOTP_SRK2                       0x5A0       /* Shadow Register for OTP Bank6 Word2 */
#define IMX_HW_OCOTP_SRK3                       0x5B0       /* Shadow Register for OTP Bank6 Word3 */
#define IMX_HW_OCOTP_SRK4                       0x5C0       /* Shadow Register for OTP Bank7 Word0 */
#define IMX_HW_OCOTP_SRK5                       0x5D0       /* Shadow Register for OTP Bank7 Word1 */
#define IMX_HW_OCOTP_SRK6                       0x5E0       /* Shadow Register for OTP Bank7 Word2 */
#define IMX_HW_OCOTP_SRK7                       0x5F0       /* Shadow Register for OTP Bank7 Word3 */
#define IMX_HW_OCOTP_SJC_RESP0                  0x600       /* Value of OTP Bank8 Word0 */
#define IMX_HW_OCOTP_SJC_RESP1                  0x610       /* Value of OTP Bank8 Word1 */
#define IMX_HW_OCOTP_USB_ID                     0x620       /* Value of OTP Bank8 Word2 */
#define IMX_HW_OCOTP_FIELD_RETURN               0x630       /* Value of OTP Bank5 Word6 */
#define IMX_HW_OCOTP_MAC_ADDR0                  0x640       /* Value of OTP Bank9 Word0 */
#define IMX_HW_OCOTP_MAC_ADDR1                  0x650       /* Value of OTP Bank9 Word1 */
#define IMX_HW_OCOTP_MAC_ADDR2                  0x660       /* Value of OTP Bank9 Word2 */
#define IMX_HW_OCOTP_SRK_REVOKE                 0x670       /* Value of OTP Bank9 Word3 */
#define IMX_HW_OCOTP_MAU_KEY0                   0x680       /* Shadow Register for OTP Bank10 Word0 */
#define IMX_HW_OCOTP_MAU_KEY1                   0x690       /* Shadow Register for OTP Bank10 Word1 */
#define IMX_HW_OCOTP_MAU_KEY2                   0x6A0       /* Shadow Register for OTP Bank10 Word2 */
#define IMX_HW_OCOTP_MAU_KEY3                   0x6B0       /* Shadow Register for OTP Bank10 Word3 */
#define IMX_HW_OCOTP_MAU_KEY4                   0x6C0       /* Shadow Register for OTP Bank11 Word0 */
#define IMX_HW_OCOTP_MAU_KEY5                   0x6D0       /* Shadow Register for OTP Bank11 Word1 */
#define IMX_HW_OCOTP_MAU_KEY6                   0x6E0       /* Shadow Register for OTP Bank11 Word2 */
#define IMX_HW_OCOTP_MAU_KEY7                   0x6F0       /* Shadow Register for OTP Bank11 Word3 */
#define IMX_HW_OCOTP_GP10                       0x780       /* Value of OTP Bank14 Word0 */
#define IMX_HW_OCOTP_GP11                       0x790       /* Value of OTP Bank14 Word1 */
#define IMX_HW_OCOTP_GP20                       0x7A0       /* Value of OTP Bank14 Word2 */
#define IMX_HW_OCOTP_GP21                       0x7B0       /* Value of OTP Bank14 Word3 */

#endif /* IMX_DDRC_H_ */
