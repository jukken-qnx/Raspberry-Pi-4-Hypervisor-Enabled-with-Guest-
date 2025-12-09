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

#ifndef IMX_SRC_H_
#define IMX_SRC_H_

/* SRC registers, offset from base address */
#define IMX_SRC_SCR                        0x00     /* SRC Reset Control Register */
#define IMX_SRC_A53RCR0                    0x04     /* A53 Reset Control Register */
#define IMX_SRC_A53RCR1                    0x08     /* A53 Reset Control Register */
#define IMX_SRC_M7RCR                      0x0C     /* M4 Reset Control Register */
#define IMX_SRC_USBOPHY1_RCR               0x20     /* USB OTG PHY1 Reset Control Register */
#define IMX_SRC_USBOPHY2_RCR               0x24     /* USB OTG PHY2 Reset Control Register */
#define IMX_SRC_MIPIPHY_RCR                0x28     /* MIPI PHY Reset Control Register */
#define IMX_SRC_PCIEPHY_RCR                0x2C     /* PCIE PHY Reset Control Register */
#define IMX_SRC_HDMI_RCR                   0x30     /* HDMI Reset Control Register */
#define IMX_SRC_DISP_RCR                   0x34     /* DISP Reset Control Register */
#define IMX_SRC_GPU_RCR                    0x40     /* GPU Reset Control Register */
#define IMX_SRC_VPU_RCR                    0x44     /* VPU Reset Control Register */
#define IMX_SRC_PCIE2_RCR                  0x48     /* PCIE2 Reset Control Register */
#define IMX_SRC_MIPIPHY1_RCR               0x4C     /* MIPI CSI1 PHY Reset Control Register */
#define IMX_SRC_MIPIPHY2_RCR               0x50     /* MIPI CSI2 PHY Reset Control Register */
#define IMX_SRC_SBMR1                      0x58     /* SRC Boot Mode Register 1 */
#define IMX_SRC_SRSR                       0x5C     /* SRC Reset Status Register */
#define IMX_SRC_SISR                       0x68     /* SRC Interrupt Status Register */
#define IMX_SRC_SIMR                       0x6C     /* SRC Interrupt Mask Register */
#define IMX_SRC_SBMR2                      0x70     /* SRC Boot Mode Register 2 */
#define IMX_SRC_GPR1                       0x74     /* SRC General Purpose Register 1 */
#define IMX_SRC_GPR2                       0x78     /* SRC General Purpose Register 2 */
#define IMX_SRC_GPR3                       0x7C     /* SRC General Purpose Register 3 */
#define IMX_SRC_GPR4                       0x80     /* SRC General Purpose Register 4 */
#define IMX_SRC_GPR5                       0x84     /* SRC General Purpose Register 5 */
#define IMX_SRC_GPR6                       0x88     /* SRC General Purpose Register 6 */
#define IMX_SRC_GPR7                       0x8C     /* SRC General Purpose Register 7 */
#define IMX_SRC_GPR8                       0x90     /* SRC General Purpose Register 8 */
#define IMX_SRC_GPR9                       0x94     /* SRC General Purpose Register 9 */
#define IMX_SRC_GPR10                      0x98     /* SRC General Purpose Register 10 */
#define IMX_SRC_DDRC_RCR                   0x1000   /* SRC DDR Controller Reset Control Register */
#define IMX_SRC_DDRC2_RCR                  0x1004   /* SRC DDRC2 Controller Reset Control Register */

#endif /* IMX_DDRC_H_ */
