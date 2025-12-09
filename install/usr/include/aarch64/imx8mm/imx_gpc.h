/*
 * $QNXLicenseC:
 * Copyright 2018 NXP
 * Copyright 2021, QNX Software Systems.
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

#ifndef IMX_GPC_H_
#define IMX_GPC_H_

/* GPC registers, offset from base address */
#define IMX_GPC_LPCR_A53_BSC                        0x00     /* Basic Low power control register of A53 platform */
#define IMX_GPC_LPCR_A53_AD                         0x04     /* Advanced Low power control register of A53 platform */
#define IMX_GPC_LPCR_M4                             0x08     /* Low power control register of CPU1 */
#define IMX_GPC_SLPCR                               0x14     /* System low power control register */
#define IMX_GPC_MST_CPU_MAPPING                     0x18     /* MASTER LPM Handshake */
#define IMX_GPC_MLPCR                               0x20     /* Memory low power control register */
#define IMX_GPC_PGC_ACK_SEL_A53                     0x24     /* PGC acknowledge signal selection of A53 platform */
#define IMX_GPC_PGC_ACK_SEL_M4                      0x28     /* PGC acknowledge signal selection of M4 platform */
#define IMX_GPC_MISC                                0x2C     /* GPC Miscellaneous register */
#define IMX_GPC_IMR1_CORE0_A53                      0x30     /* IRQ masking register 1 of A53 core0 */
#define IMX_GPC_IMR2_CORE0_A53                      0x34     /* IRQ masking register 2 of A53 core0 */
#define IMX_GPC_IMR3_CORE0_A53                      0x38     /* IRQ masking register 3 of A53 core0 */
#define IMX_GPC_IMR4_CORE0_A53                      0x3C     /* IRQ masking register 4 of A53 core0 */
#define IMX_GPC_IMR1_CORE1_A53                      0x40     /* IRQ masking register 1 of A53 core1 */
#define IMX_GPC_IMR2_CORE1_A53                      0x44     /* IRQ masking register 2 of A53 core1 */
#define IMX_GPC_IMR3_CORE1_A53                      0x48     /* IRQ masking register 3 of A53 core1 */
#define IMX_GPC_IMR4_CORE1_A53                      0x4C     /* IRQ masking register 4 of A53 core1 */
#define IMX_GPC_IMR1_M4                             0x50     /* IRQ masking register 1 of M4 */
#define IMX_GPC_IMR2_M4                             0x54     /* IRQ masking register 2 of M4 */
#define IMX_GPC_IMR3_M4                             0x58     /* IRQ masking register 3 of M4 */
#define IMX_GPC_IMR4_M4                             0x5C     /* IRQ masking register 4 of M4 */
#define IMX_GPC_ISR1_A53                            0x70     /* IRQ status register 1 of A53 */
#define IMX_GPC_ISR2_A53                            0x74     /* IRQ status register 2 of A53 */
#define IMX_GPC_ISR3_A53                            0x78     /* IRQ status register 3 of A53 */
#define IMX_GPC_ISR4_A53                            0x7C     /* IRQ status register 4 of A53 */
#define IMX_GPC_ISR1_M4                             0x80     /* IRQ status register 1 of M4 */
#define IMX_GPC_ISR2_M4                             0x84     /* IRQ status register 2 of M4 */
#define IMX_GPC_ISR3_M4                             0x88     /* IRQ status register 3 of M4 */
#define IMX_GPC_ISR4_M4                             0x8C     /* IRQ status register 4 of M4 */
#define IMX_GPC_SLTn_CFG(n)                         (0xB0 + (4 * n)) /* Slot configure register for A53 core */
#define IMX_GPC_PGC_CPU_0_1_MAPPING                 0xEC     /* PGC CPU mapping */
#define IMX_GPC_CPU_PGC_SW_PUP_REQ                  0xF0     /* CPU PGC software power up trigger */
#define IMX_GPC_MIX_PGC_SW_PUP_REQ                  0xF4     /* MIX PGC software power up trigger */
#define IMX_GPC_PU_PGC_SW_PUP_REQ                   0xF8     /* PU PGC software up trigger */
#define IMX_GPC_CPU_PGC_SW_PDN_REQ                  0xFC     /* CPU PGC software down trigger */
#define IMX_GPC_MIX_PGC_SW_PDN_REQ                  0x100    /* MIX PGC software power down trigger */
#define IMX_GPC_PU_PGC_SW_PDN_REQ                   0x104    /* PU PGC software down trigger */
#define IMX_GPC_LPCR_A53_BSC2                       0x108    /* Basic Low power control register of A53 platform */
#define IMX_GPC_CPU_PGC_PUP_STATUS1                 0x130    /* CPU PGC software up trigger status1 */
#define IMX_GPC_A53_MIX_PGC_PUP_STATUS(n)           (0x134 + (4 * n)) /* A53 MIX software up trigger status register */
#define IMX_GPC_M4_MIX_PGC_PUP_STATUS(n)            (0x140 + (4 * n)) /* M4 MIX PGC software up trigger status register */
#define IMX_GPC_A53_PU_PGC_PUP_STATUS(n)            (0x14C + (4 * n)) /* A53 PU software up trigger status register */
#define IMX_GPC_M4_PU_PGC_PUP_STATUS(n)             (0x158 + (4 * n)) /* M4 PU PGC software up trigger status register */
#define IMX_GPC_CPU_PGC_PDN_STATUS1                 0x170    /* CPU PGC software dn trigger status1 */
#define IMX_GPC_A53_MIX_PGC_PDN_STATUS(n)           (0x174 + (4 * n)) /* A53 MIX software down trigger status register */
#define IMX_GPC_M4_MIX_PGC_PDN_STATUS(n)            (0x180 + (4 * n)) /* M4 MIX PGC software power down trigger status register */
#define IMX_GPC_A53_PU_PGC_PDN_STATUS(n)            (0x18C + (4 * n)) /* A53 PU PGC software down trigger status */
#define IMX_GPC_M4_PU_PGC_PDN_STATUS(n)             (0x198 + (4 * n)) /* M4 PU PGC software down trigger status */
#define IMX_GPC_A53_MIX_PDN_FLG                     0x1B0     /* A53 MIX PDN FLG */
#define IMX_GPC_A53_PU_PDN_FLG                      0x1B4     /* A53 PU PDN FLG */
#define IMX_GPC_M4_MIX_PDN_FLG                      0x1B8     /* M4 MIX PDN FLG */
#define IMX_GPC_M4_PU_PDN_FLG                       0x1BC     /* M4 PU PDN FLG */
#define IMX_GPC_IMR1_CORE2_A53                      0x1C0     /* IRQ masking register 1 of A53 core2 */
#define IMX_GPC_IMR2_CORE2_A53                      0x1C4     /* IRQ masking register 2 of A53 core2 */
#define IMX_GPC_IMR3_CORE2_A53                      0x1C8     /* IRQ masking register 3 of A53 core2 */
#define IMX_GPC_IMR4_CORE2_A53                      0x1CC     /* IRQ masking register 4 of A53 core2 */
#define IMX_GPC_IMR1_CORE3_A53                      0x1D0     /* IRQ masking register 1 of A53 core3 */
#define IMX_GPC_IMR2_CORE3_A53                      0x1D4     /* IRQ masking register 2 of A53 core3 */
#define IMX_GPC_IMR3_CORE3_A53                      0x1D8     /* IRQ masking register 3 of A53 core3 */
#define IMX_GPC_IMR4_CORE3_A53                      0x1DC     /* IRQ masking register 4 of A53 core3 */
#define IMX_GPC_ACK_SEL_A53_PU                      0x1E0     /* PGC acknowledge signal selection of A53 platform for PUs */
#define IMX_GPC_ACK_SEL_M4_PU                       0x1E4     /* PGC acknowledge signal selection of M4 platform for PUs */
#define IMX_GPC_SLT_CFG(n)                          (0x1E8 + (4 * n)) /* Slot configure register for A53 core */
#define IMX_GPC_PU_PWRHSK                           0x1FC     /* Power handshake register */
#define IMX_GPC_SLT_CFG_PU(n)                       (0x200 + (4 * n)) /* Slot configure register for PUs */

/* Requests: PUP, PDN */
#define IMX_GPC_PU_PGC_SW_REQ_MIPI_DSI_SHIFT        (0)
#define IMX_GPC_PU_PGC_SW_REQ_MIPI_DSI_MASK         (1 << IMX_GPC_PU_PGC_SW_REQ_MIPI_DSI_SHIFT)
#define IMX_GPC_PU_PGC_SW_REQ_PCIE_SHIFT            (1)
#define IMX_GPC_PU_PGC_SW_REQ_PCIE_MASK             (1 << IMX_GPC_PU_PGC_SW_REQ_PCIE_SHIFT)
#define IMX_GPC_PU_PGC_SW_REQ_USB_OTG1_SHIFT        (2)
#define IMX_GPC_PU_PGC_SW_REQ_USB_OTG1_MASK         (1 << IMX_GPC_PU_PGC_SW_REQ_USB_OTG1_SHIFT)
#define IMX_GPC_PU_PGC_SW_REQ_USB_OTG2_SHIFT        (3)
#define IMX_GPC_PU_PGC_SW_REQ_USB_OTG2_MASK         (1 << IMX_GPC_PU_PGC_SW_REQ_USB_OTG2_SHIFT)
#define IMX_GPC_PU_PGC_SW_REQ_DDR1_SHIFT            (5)
#define IMX_GPC_PU_PGC_SW_REQ_DDR1_MASK             (1 << IMX_GPC_PU_PGC_SW_REQ_DDR1_SHIFT)
#define IMX_GPC_PU_PGC_SW_REQ_GPU_2D_SHIFT          (6)
#define IMX_GPC_PU_PGC_SW_REQ_GPU_2D_MASK           (1 << IMX_GPC_PU_PGC_SW_REQ_GPU_2D_SHIFT)
#define IMX_GPC_PU_PGC_SW_REQ_GPUMIX_SHIFT          (7)
#define IMX_GPC_PU_PGC_SW_REQ_GPUMIX_MASK           (1 << IMX_GPC_PU_PGC_SW_REQ_GPUMIX_SHIFT)
#define IMX_GPC_PU_PGC_SW_REQ_VPUMIX_SHIFT          (8)
#define IMX_GPC_PU_PGC_SW_REQ_VPUMIX_MASK           (1 << IMX_GPC_PU_PGC_SW_REQ_VPUMIX_SHIFT)
#define IMX_GPC_PU_PGC_SW_REQ_GPU_3D_SHIFT          (9)
#define IMX_GPC_PU_PGC_SW_REQ_GPU_3D_MASK           (1 << IMX_GPC_PU_PGC_SW_REQ_GPU_3D_SHIFT)
#define IMX_GPC_PU_PGC_SW_REQ_DISPMIX_SHIFT         (10)
#define IMX_GPC_PU_PGC_SW_REQ_DISPMIX_MASK          (1 << IMX_GPC_PU_PGC_SW_REQ_DISPMIX_SHIFT)
#define IMX_GPC_PU_PGC_SW_REQ_VPU_G1_SHIFT          (11)
#define IMX_GPC_PU_PGC_SW_REQ_VPU_G1_MASK           (1 << IMX_GPC_PU_PGC_SW_REQ_VPU_G1_SHIFT)
#define IMX_GPC_PU_PGC_SW_REQ_VPU_G2_SHIFT          (12)
#define IMX_GPC_PU_PGC_SW_REQ_VPU_G2_MASK           (1 << IMX_GPC_PU_PGC_SW_REQ_VPU_G2_SHIFT)
#define IMX_GPC_PU_PGC_SW_REQ_VPU_H1_SHIFT          (13)
#define IMX_GPC_PU_PGC_SW_REQ_VPU_H1_MASK           (1 << IMX_GPC_PU_PGC_SW_REQ_VPU_H1_SHIFT)

/* GPC PGC registers, offset from base address */

#define IMX_GPC_PGC_nCTRL(n)                        (0x800 + (64 * n)) /* GPC PGC Control Register */
#define IMX_GPC_PGC_nPUPSCR(n)                      (0x804 + (64 * n)) /* GPC PGC Up Sequence Control Register */
#define IMX_GPC_PGC_nPDNSCR(n)                      (0x808 + (64 * n)) /* GPC PGC Down Sequence Control Register */
#define IMX_GPC_PGC_nSR(n)                          (0x80C + (64 * n)) /* GPC PGC Status Register */
#define IMX_GPC_PGC_nAUXSW(n)                       (0x810 + (64 * n)) /* GPC PGC Auxiliary Power Switch Control Register */
#define IMX_GPC_PGC_CTRL                            0xA00    /* GPC PGC Control Register */
#define IMX_GPC_PGC_PUPSCR                          0xA04    /* GPC PGC Up Sequence Control Register */
#define IMX_GPC_PGC_PDNSCR                          0xA08    /* GPC PGC Down Sequence Control Register */
#define IMX_GPC_PGC_SR                              0xA0C    /* GPC PGC Status Register */
#define IMX_GPC_PGC_AUXSW                           0xA10    /* GPC PGC Auxiliary Power Switch Control Register */




/*
 * IMX_GPC_PU_PWRHSK Control Divider Register bits *
 */
#define IMX_GPC_PU_PWRHSK_DDR1_CORE_CSYSREQ_SHIFT   (0)             /* DDR1 controller Hardware Low-Power Request */
#define IMX_GPC_PU_PWRHSK_DDR1_CORE_CSYSREQ_MASK    (0x01 << 0)
#define IMX_GPC_PU_PWRHSK_DDR1_AXI_CSYSREQ_SHIFT    (1)             /* DDR1 AXI Low-Power Request */
#define IMX_GPC_PU_PWRHSK_DDR1_AXI_CSYSREQ_MASK     (0x01 << 1)
#define IMX_GPC_PU_PWRHSK_NOC2DDR_PWRDNREQN_SHIFT   (2)             /* NOC2DDR ADB400 power down request. */
#define IMX_GPC_PU_PWRHSK_NOC2DDR_PWRDNREQN_MASK    (0x01 << 2)
#define IMX_GPC_PU_PWRHSK_HSIOMIX2NOC_PWRDNREQN_SHIFT (5)           /* HSIOMIX2NOC ADB400 power down request */
#define IMX_GPC_PU_PWRHSK_HSIOMIX2NOC_PWRDNREQN_MASK (0x01 << 5)
#define IMX_GPC_PU_PWRHSK_NOC2HSIOMIX_ADBS_PWRDNREQN_SHIFT (6)      /* NOC2HSIOMIX_ADBS_PWRDNREQN power down request */
#define IMX_GPC_PU_PWRHSK_NOC2HSIOMIX_ADBS_PWRDNREQN_MASK  (0x01 << 6)
#define IMX_GPC_PU_PWRHSK_DISPMIX_PWRDNREQN_SHIFT   (7)             /* DISPMIX2NOC power down request. */
#define IMX_GPC_PU_PWRHSK_DISPMIX_PWRDNREQN_MASK    (0x01 << 7)
#define IMX_GPC_PU_PWRHSK_VPUPMIX_PWRDNREQN_SHIFT   (8)             /* VPUPMIX_PWRDNREQN power down request. */
#define IMX_GPC_PU_PWRHSK_VPUPMIX_PWRDNREQN_MASK    (0x01 << 8)
#define IMX_GPC_PU_PWRHSK_GPUPMIX2NOC_3D_PWRDNREQN_SHIFT (9)        /* GPUPMIX2NOC_3D_PWRDNREQN power down request. */
#define IMX_GPC_PU_PWRHSK_GPUPMIX2NOC_3D_PWRDNREQN_MASK  (0x01 << 9)
#define IMX_GPC_PU_PWRHSK_GPUPMIX2NOC_2D_PWRDNREQN_SHIFT (10)       /* GPUPMIX2NOC_2D_PWRDNREQN power down request. */
#define IMX_GPC_PU_PWRHSK_GPUPMIX2NOC_2D_PWRDNREQN_MASK  (0x01 << 10)
#define IMX_GPC_PU_PWRHSK_NOC2GPUPMIX_PWRDNREQN_SHIFT (11)          /* NOC2GPUPMIX_PWRDNREQN power down request. */
#define IMX_GPC_PU_PWRHSK_NOC2GPUPMIX_PWRDNREQN_MASK (0x01 << 11)
#define IMX_GPC_PU_PWRHSK_DDR1_CORE_CSYSACK_SHIFT   (16)            /* DDR1 controller Hardware Low_Power ack */
#define IMX_GPC_PU_PWRHSK_DDR1_CORE_CSYSACK_MASK    (0x01 << 16)
#define IMX_GPC_PU_PWRHSK_DDR1_CORE_CACTIVE_SHIFT   (17)            /* DDR1 controller Hardware Low-Power Clock active */
#define IMX_GPC_PU_PWRHSK_DDR1_CORE_CACTIVE_MASK    (0x01 << 17)
#define IMX_GPC_PU_PWRHSK_DDR1_AXI_CSYSACK_SHIFT    (18)            /* DDR1 AXI Low-Power Request ack */
#define IMX_GPC_PU_PWRHSK_DDR1_AXI_CSYSACK_MASK     (0x01 << 18)
#define IMX_GPC_PU_PWRHSK_DDR1_AXI_CACTIVE_SHIFT    (19)            /* DDR1 AXI Clock Active */
#define IMX_GPC_PU_PWRHSK_DDR1_AXI_CACTIVE_MASK     (0x01 << 19)
#define IMX_GPC_PU_PWRHSK_NOC2DDR1_PWRDNACKN_SHIFT  (20)            /* NOC2DDR ADB400 power down ack */
#define IMX_GPC_PU_PWRHSK_NOC2DDR1_PWRDNACKN_MASK   (0x01 << 20)
#define IMX_GPC_PU_PWRHSK_SUPERMIX2NOC_PWRDNACKN_SHIFT (21)         /* SUPERMIX2NOC ADB400 power down ack. */
#define IMX_GPC_PU_PWRHSK_SUPERMIX2NOC_PWRDNACKN_MASK (0x01 << 21)
#define IMX_GPC_PU_PWRHSK_NOC2SUPERMIX_PWRDNACKN_SHIFT (22)         /* NOC2SUPERMIX ADB400 power down ack. */
#define IMX_GPC_PU_PWRHSK_NOC2SUPERMIX_PWRDNACKN_MASK (0x01 << 22)
#define IMX_GPC_PU_PWRHSK_HSIOMIX2NOC_PWRDNACKN_SHIFT (23)          /* HSIOMIX2NOC ADB400 power down ack. */
#define IMX_GPC_PU_PWRHSK_HSIOMIX2NOC_PWRDNACKN_MASK (0x01 << 23)
#define IMX_GPC_PU_PWRHSK_DISPMIX2NOC_PWRDNACKN_SHIFT (25)          /* DISPMIX2NOC ADB400 power down ack. */
#define IMX_GPC_PU_PWRHSK_DISPMIX2NOC_PWRDNACKN_MASK (0x01 << 25)
#define IMX_GPC_PU_PWRHSK_VPUMIX_PWRDNACKN_SHIFT    (26)            /* VPUMIX_PWRDNACKN power down ack. */
#define IMX_GPC_PU_PWRHSK_VPUMIX_PWRDNACKN_MASK     (0x01 << 26)
#define IMX_GPC_PU_PWRHSK_GPUMIX2NOC_PWRDNACKN_SHIFT (27)           /* GPUMIX2NOC ADB400 power down ack. */
#define IMX_GPC_PU_PWRHSK_GPUMIX2NOC_PWRDNACKN_MASK (1 << 27)
#define IMX_GPC_PU_PWRHSK_GPUMIX2NOC_2D_PWRDNACKN_SHIFT (28)        /* GPUMIX2NOC ADB400 power down ack. */
#define IMX_GPC_PU_PWRHSK_GPUMIX2NOC_2D_PWRDNACKN_MASK (1 << 28)
#define IMX_GPC_PU_PWRHSK_NOC2GPUMIX_PWRDNACKN_SHIFT (29)           /* NOC2GPUMIX_PWRDNACKN power down ack. */
#define IMX_GPC_PU_PWRHSK_NOC2GPUMIX_PWRDNACKN_MASK (1 << 29)

/* PU type power domain */
#define IMX_GPC_PGC_PU_nCTRL(n)                     (0xC00 + (64 * n)) /* GPC PGC Control Register */
#define IMX_GPC_PGC_PU_nPUPSCR(n)                   (0xC04 + (64 * n)) /* GPC PGC Up Sequence Control Register */
#define IMX_GPC_PGC_PU_nPDNSCR(n)                   (0xC08 + (64 * n)) /* GPC PGC Down Sequence Control Register */
#define IMX_GPC_PGC_PU_nSR(n)                       (0xC0C + (64 * n)) /* GPC PGC Status Register */
#define IMX_GPC_PGC_PU_nAUXSW(n)                    (0xC10 + (64 * n)) /* GPC PGC Auxiliary Power Switch Control Register */
#define IMX_GPC_PGC_PU_nCTRL_PCR_SHIFT              (0)
#define IMX_GPC_PGC_PU_nCTRL_PCR_MASK               (0x01)

/* PU power domain indexes - see IMX_GPC_PGC_PU_nCTRL(n) */
#define IMX_GPC_PGC_PU_MIPI_DSI_PHY                 0
#define IMX_GPC_PGC_PU_PCIE1_PHY                    1
#define IMX_GPC_PGC_PU_USB_OTG1                     2
#define IMX_GPC_PGC_PU_USB_OTG2                     3
#define IMX_GPC_PGC_PU_Reserved                     4
#define IMX_GPC_PGC_PU_DDR1                         5
#define IMX_GPC_PGC_PU_GPU_2D                       6
#define IMX_GPC_PGC_PU_GPUMIX                       7
#define IMX_GPC_PGC_PU_VPUMIX                       8
#define IMX_GPC_PGC_PU_GPU_3D                       9
#define IMX_GPC_PGC_PU_DISPMIX                     10
#define IMX_GPC_PGC_PU_VPU_G1                      11
#define IMX_GPC_PGC_PU_VPU_G2                      12
#define IMX_GPC_PGC_PU_VPU_H1                      13

/* ATF PU power domain indexes - imx_sec_firmware_psci function */
#define ATF_PU_HSIOMIX                              0UL
#define ATF_PU_PCIE                                 1UL
#define ATF_PU_OTG1                                 2UL
#define ATF_PU_OTG2                                 3UL
#define ATF_PU_GPUMIX                               4UL
#define ATF_PU_VPUMIX                               5UL
#define ATF_PU_VPU_G1                               6UL
#define ATF_PU_VPU_G2                               7UL
#define ATF_PU_VPU_H1                               8UL
#define ATF_PU_DISPMIX                              9UL
#define ATF_PU_MIPI                                10UL
/* below two domain only for ATF internal use */
#define ATF_PU_GPU2D                               11UL
#define ATF_PU_GPU3D                               12UL

#endif
