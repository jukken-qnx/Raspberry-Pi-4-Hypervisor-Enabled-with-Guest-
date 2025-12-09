/*
 * Copyright 2017, 2020-2024 BlackBerry Limited.
 * Copyright 2017, 2021 Renesas Electronics Corporation.
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

#include <errno.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <hw/inout.h>
#include <sys/mman.h>
#include <internal.h>
#include <sys/syspage.h>
#include <inttypes.h>
#include <stdio.h>


#ifdef SDIO_HC_RCAR_SDMMC
#include "rcar.h"
#include <soc/renesas/rcar/gen4/r-car-gen4.h>
#include <soc/renesas/rcar/common/rcar.h>
#ifdef RCAR_CPG
#include <hw/rcar_cpg.h>
#endif

#if defined(VARIANT_ipmmu)
#include <hw/rcar_ipmmu-api.h>
TAILQ_HEAD(ipmmu_obj_head, ipmmu_obj) ipmmu_tailq_head;
#endif /* defined(VARIANT_ipmmu) */

#define RCAR_SDMMC_SCC_DEBUG

#ifdef RCAR_CPG
static cpg_mgr_funcs_t cpg_hwfuncs;
#endif

static int rcar_sdmmc_reset(sdio_hc_t *hc);
static int rcar_sdmmc_dma_setup(sdio_hc_t *hc, sdio_cmd_t *cmd);
static void rcar_sdmmc_dma_start( sdio_hc_t *hc, sdio_cmd_t *cmd);
static void rcar_sdmmc_dma_cmplt( sdio_hc_t *hc, sdio_cmd_t *cmd, int error);
static void rcar_sdmmc_hs400_adjust_enable(sdio_hc_t *hc);
static void rcar_sdmmc_hs400_adjust_disable(sdio_hc_t *hc);
static void rcar_sdmmc_scc_hs400_reset(sdio_hc_t *hc);
static int rcar_sdmmc_scc_error_check(sdio_hc_t *hc, uint32_t info2);
static void rcar_sdmmc_disable_scc(sdio_hc_t *hc);
static void rcar_sdmmc_scc_reset(sdio_hc_t *hc);

static void rcar_sdmmc_debug(sdio_hc_t * const hc, const char * const title)
{
#ifdef RCAR_SDMMC_SCC_DEBUG
    const rcar_sdmmc_t    * const sdmmc = hc->cs_hdl;

    sdio_slogf(_SLOGC_SDIODI, _SLOG_INFO, hc->cfg.verbosity, 1, "   *********   %s   ********  ", title);
    sdio_slogf(_SLOGC_SDIODI, _SLOG_INFO, hc->cfg.verbosity, 1, "   MMC_SCC_TAPSET      = 0x%08X", sdmmc_read(sdmmc->vbase, MMC_SCC_TAPSET));
    sdio_slogf(_SLOGC_SDIODI, _SLOG_INFO, hc->cfg.verbosity, 1, "   MMC_SCC_DTCNTL      = 0x%08X", sdmmc_read(sdmmc->vbase, MMC_SCC_DTCNTL));
    sdio_slogf(_SLOGC_SDIODI, _SLOG_INFO, hc->cfg.verbosity, 1, "   MMC_SCC_RVSCNTL     = 0x%08X", sdmmc_read(sdmmc->vbase, MMC_SCC_RVSCNTL));
    sdio_slogf(_SLOGC_SDIODI, _SLOG_INFO, hc->cfg.verbosity, 1, "   MMC_SD_CLK_CTRL     = 0x%08X", sdmmc_read(sdmmc->vbase, MMC_SD_CLK_CTRL));
    sdio_slogf(_SLOGC_SDIODI, _SLOG_INFO, hc->cfg.verbosity, 1, "   MMC_SCC_DT2FF       = 0x%08X", sdmmc_read(sdmmc->vbase, MMC_SCC_DT2FF));
    sdio_slogf(_SLOGC_SDIODI, _SLOG_INFO, hc->cfg.verbosity, 1, "   MMC_SDIF_MODE       = 0x%08X", sdmmc_read(sdmmc->vbase, MMC_SDIF_MODE));
    sdio_slogf(_SLOGC_SDIODI, _SLOG_INFO, hc->cfg.verbosity, 1, "   MMC_SCC_TMPPORT2    = 0x%08X", sdmmc_read(sdmmc->vbase, MMC_SCC_TMPPORT2));
    sdio_slogf(_SLOGC_SDIODI, _SLOG_INFO, hc->cfg.verbosity, 1, "   MMC_SCC_CKSEL       = 0x%08X", sdmmc_read(sdmmc->vbase, MMC_SCC_CKSEL));

    if ((sdmmc->adjust_hs400_enable != 0) && (hc->timing == TIMING_HS400)) {
        sdio_slogf(_SLOGC_SDIODI, _SLOG_INFO, hc->cfg.verbosity, 1, "   MMC_SCC_TMPPORT3    = 0x%08X", sdmmc_read(sdmmc->vbase, MMC_SCC_TMPPORT3));
        sdio_slogf(_SLOGC_SDIODI, _SLOG_INFO, hc->cfg.verbosity, 1, "   MMC_SCC_TMPPORT4    = 0x%08X", sdmmc_read(sdmmc->vbase, MMC_SCC_TMPPORT4));
        sdio_slogf(_SLOGC_SDIODI, _SLOG_INFO, hc->cfg.verbosity, 1, "   MMC_SCC_TMPPORT6    = 0x%08X", sdmmc_read(sdmmc->vbase, MMC_SCC_TMPPORT6));
        sdio_slogf(_SLOGC_SDIODI, _SLOG_INFO, hc->cfg.verbosity, 1, "   MMC_SCC_TMPPORT7    = 0x%08X", sdmmc_read(sdmmc->vbase, MMC_SCC_TMPPORT7));
    }

#endif
}

static int rcar_sdmmc_intr_event(sdio_hc_t * const hc)
{
    rcar_sdmmc_t    *sdmmc;
    sdio_cmd_t      *cmd;
    uint32_t        mask1, mask2;
    uint32_t        stat1, stat2, dmasta;
    uint32_t        cs = CS_CMD_INPROG;

    sdmmc = hc->cs_hdl;

    mask1 = ~sdmmc_read(sdmmc->vbase, MMC_SD_INFO1_MASK);
    mask2 = ~sdmmc_read(sdmmc->vbase, MMC_SD_INFO2_MASK);
    stat1 =  sdmmc_read(sdmmc->vbase, MMC_SD_INFO1) & mask1;
    stat2 =  sdmmc_read(sdmmc->vbase, MMC_SD_INFO2) & mask2;
    dmasta =  sdmmc_read(sdmmc->vbase, MMC_DM_CM_INFO1);

    /* Clear interrupt status */
    sdmmc_write(sdmmc->vbase, MMC_SD_INFO1, ~stat1);
    sdmmc_write(sdmmc->vbase, MMC_SD_INFO2, ~stat2);

    /*
     * Card insertion and card removal events
     */
    if (stat1 & (SDH_INFO1_INST | SDH_INFO1_RMVL)) {
        sdio_hc_event(hc, HC_EV_CD);
    }

    /* no command ? */
    cmd = hc->wspc.cmd;
    if (cmd == NULL) {
        return (EOK);
    }

    /* Start DMA ? */
    if ((stat1 & SDH_INFO1_RE) && !(((stat2 & SDH_INFO2_CRCE) && (cmd->flags & SCF_RSP_CRC)) ||
                                     (stat2 & SDH_INFO2_CMDE) ||
                                     (stat2 & SDH_INFO2_ENDE) ||
                                     (stat2 & SDH_INFO2_RTO))) {
        if (cmd->flags & SCF_CTYPE_ADTC) {
            rcar_sdmmc_dma_start(hc, cmd);
        }
    }

    /* Check of errors */
    if (stat2 & (SDH_INFO2_ALL_ERR)) {
        /* Description of bit7 of SD_INFO2 register:
         * If the data timeout (ERR3) is set but the response timeout (ERR6) is not set after the Erase command has been issued,
         * the end of the Erase sequence (DAT0 = 1) is confirmed by polling DAT0 */
        if ((cmd->opcode == MMC_ERASE) && ((stat2 & (SDH_INFO2_DTO | SDH_INFO2_RTO)) == SDH_INFO2_DTO)) {
            int timeout = 10*1000*1000; /* wait up to 10s for DAT0=1, the timeout value can depend on eMMC device */
            while (timeout-- > 0) {
                if (sdmmc_read(sdmmc->vbase, MMC_SD_INFO2) & SDH_INFO2_DAT0) {
                    cs = CS_CMD_CMP;
                    stat2 &= ~SDH_INFO2_DTO;
                    break;
                }
                nanospin_ns(1000);
            }
        }

        if (cs == CS_CMD_INPROG) {
            uint32_t    ests1, ests2;

            /* DMA error processing? */
            if ((sdmmc->flags & OF_DMA_ACTIVE) && (cmd->flags & SCF_CTYPE_ADTC)) {
                uint32_t      dma_info1, dma_info2;

                dma_info1 = sdmmc_read(sdmmc->vbase, MMC_DM_CM_INFO1);
                dma_info2 = sdmmc_read(sdmmc->vbase, MMC_DM_CM_INFO2);

                if (dma_info2 & (DM_INFO2_DTRAN_ERR0 | DM_INFO2_DTRAN_ERR1)) {
                    sdio_slogf( _SLOGC_SDIODI, _SLOG_ERROR, hc->cfg.verbosity, 0, "%s: ERROR in SDHI%d, CMD %d, DMA info1 0x%X, DMA info2 0x%X",
                                        __func__, sdmmc->chan_idx, cmd->opcode, dma_info1, dma_info2 );

                    rcar_sdmmc_dma_cmplt(hc, cmd, 1);
                }
            }

            ests1 = sdmmc_read(sdmmc->vbase, MMC_SD_ERR_STS1);
            ests2 = sdmmc_read(sdmmc->vbase, MMC_SD_ERR_STS2);

            sdio_slogf(_SLOGC_SDIODI, _SLOG_ERROR, hc->cfg.verbosity, ((cmd->opcode == MMC_SEND_TUNING_BLOCK) || (cmd->opcode == SD_SEND_TUNING_BLOCK)) ? 1 : 0,
                            "%s: ERROR in SDHI%d, CMD %d, INFO 0x%x-0x%x, STS 0x%x-0x%x",
                            __func__, sdmmc->chan_idx, cmd->opcode, stat1, stat2,
                            sdmmc_read(sdmmc->vbase, MMC_SD_ERR_STS1), sdmmc_read(sdmmc->vbase, MMC_SD_ERR_STS2));

            if (stat2 & SDH_INFO2_DTO) {
                cs = CS_DATA_TO_ERR;
            }
            if (ests1 & (1 << 11)) {
                cs = CS_DATA_CRC_ERR;
            }
            if (ests1 & (1 << 10)) {
                cs = CS_DATA_CRC_ERR;
            }
            if (ests1 & ((1 << 8) | (1 << 9))) {
                cs = CS_CMD_CRC_ERR;
            }
            if (ests1 & (1 << 5)) {
                cs = CS_CMD_END_ERR;
            }
            if (ests1 & (1 << 4)) {
                cs = CS_DATA_END_ERR;
            }
            if (ests1 & ((1 << 1) | (1 << 0))) {
                cs = CS_CMD_IDX_ERR;
            }
            if (ests2 & (1 << 5)) {
                cs = CS_DATA_CRC_ERR;
            }
            if (ests2 & ((1 << 5) | (1 << 4))) {
                cs = CS_DATA_TO_ERR;
            }
            if (ests2 & ((1 << 1) | (1 << 0))) {
                cs = CS_CMD_TO_ERR;
            }
            if (!cs) {
                cs = CS_CMD_CMP_ERR;
            }
        }

    } else {
        /* End of command */
        if (stat1 & SDH_INFO1_RE) {
            if (!(cmd->flags & SCF_CTYPE_ADTC)){
                cs = CS_CMD_CMP;

                if ((cmd->flags & SCF_RSP_136)) {
                    uint32_t    * const resp = &cmd->rsp[0];
                        resp[0] = sdmmc_read(sdmmc->vbase, MMC_SD_RSP76);
                        resp[1] = sdmmc_read(sdmmc->vbase, MMC_SD_RSP54);
                        resp[2] = sdmmc_read(sdmmc->vbase, MMC_SD_RSP32);
                        resp[3] = sdmmc_read(sdmmc->vbase, MMC_SD_RSP10);

                        resp[0] = (resp[0] << 8) | (resp[1] >> 24);
                        resp[1] = (resp[1] << 8) | (resp[2] >> 24);
                        resp[2] = (resp[2] << 8) | (resp[3] >> 24);
                        resp[3] = (resp[3] << 8);
                } else if ((cmd->flags & SCF_RSP_PRESENT)) {
                    cmd->rsp[0] = sdmmc_read(sdmmc->vbase, MMC_SD_RSP10);
                }
                else {
                    // nothing
                }
            }
        } else if (stat1 & SDH_INFO1_AE) {
            /* End of data transfer */
            cmd->rsp[0] = sdmmc_read(sdmmc->vbase, MMC_SD_RSP10);
            sdmmc->flags |= OF_DATA_DONE;

            if ((sdmmc->flags & (OF_DATA_DONE | OF_DMA_DONE))==(OF_DATA_DONE | OF_DMA_DONE)) {
                cs = CS_CMD_CMP;
            }
        }
        else {
            // nothing
        }
    }

    if (sdmmc->flags & OF_DMA_ACTIVE) {
        if (dmasta & ( (cmd->flags & SCF_DIR_IN) ? DM_INFO1_DTRAN_END1(sdmmc->dma_tranend1) : DM_INFO1_DTRAN_END0)) {
            /* Clear DMA status */
            sdmmc_write(sdmmc->vbase, MMC_DM_CM_INFO1, 0);

            sdmmc->flags |= OF_DMA_DONE;
            if ((sdmmc->flags & (OF_DATA_DONE | OF_DMA_DONE))==(OF_DATA_DONE | OF_DMA_DONE)) {
                cs = CS_CMD_CMP;
            }
        }
    }

    if (stat1 & SDH_INFO1_RMVL) {
        cs = CS_CARD_REMOVED;
    }

    if (cs != CS_CMD_INPROG) {
        /* Command operation completion: Disable interrupts */
        sdmmc_write(sdmmc->vbase, MMC_SD_INFO1_MASK, (uint32_t)(~(SDH_INFO1_RMVL | SDH_INFO1_INST)));
        sdmmc_write(sdmmc->vbase, MMC_SD_INFO2_MASK, sdmmc_read(sdmmc->vbase, MMC_SD_INFO2_MASK) | SDH_INFO2_ALL_ERR);

        if ((sdmmc->flags & OF_DMA_ACTIVE) && (cmd->flags & SCF_CTYPE_ADTC)) {
            rcar_sdmmc_dma_cmplt(hc, cmd, ((cs != CS_CMD_CMP) ? 1 : 0));
        }

        if ((sdmmc->need_adjust_hs400 != 0) && (cmd->opcode == MMC_SEND_STATUS)) {
            if (cmd->rsp[0] & CDS_READY_FOR_DATA) {
                rcar_sdmmc_hs400_adjust_enable(hc);
            }
        }

        /* Check SCC status */
        if (rcar_sdmmc_scc_error_check(hc, stat2)) {
            rcar_sdmmc_scc_reset(hc);

            /* This is to perform retune */
            sdio_hc_event(hc, HC_EV_TUNE);

            sdmmc->tuning_status = RCAR_NEED_RETUNING;
        }

        sdio_cmd_cmplt(hc, cmd, cs);
    }

    return (EOK);
}

#if defined(VARIANT_ipmmu)
static void rcar_ipmmu_set_mru_obj(struct ipmmu_obj *obj)
{
    struct ipmmu_obj *cur_head_obj;

    /* Most Recently Used object should be on the
     * head of the tailq for optimized lookup/access */
    cur_head_obj = TAILQ_FIRST(&ipmmu_tailq_head);
    if (cur_head_obj == obj) {
        return; /* nothing to do */
    }
    TAILQ_REMOVE(&ipmmu_tailq_head, obj, ipmmu_tailq);
    TAILQ_INSERT_HEAD(&ipmmu_tailq_head, obj, ipmmu_tailq);
}

static struct ipmmu_obj * rcar_ipmmu_remap_obj(rcar_sdmmc_t *sdmmc, struct ipmmu_obj *obj, uint64_t paddr, uint32_t len)
{
    int ret;

    /* Unmap address
     * Need to figure out: What is correct behavior if unmap fails?
     *       Ignoring any error for now */
    ipmmu_unmap(sdmmc->ipmmu_handle, obj->paddr, obj->iova, obj->len);
    /* Remap address to get new iova */
    ret = ipmmu_map(sdmmc->ipmmu_handle, paddr, len, &obj->iova);
    if (ret != EOK) {
        /* Remove here instead of doing this outside */
        TAILQ_REMOVE(&ipmmu_tailq_head, obj, ipmmu_tailq);
        sdmmc->ipmmu_obj_count--;
        free(obj);
        return NULL;
    }
    /* Reuse the ipmmu object, just update the contents */
    obj->paddr = paddr;
    obj->len = len;

    return obj;
}

static struct ipmmu_obj * rcar_ipmmu_create_obj(rcar_sdmmc_t *sdmmc, uint64_t paddr, uint32_t len)
{
    struct ipmmu_obj *obj;
    int ret;

    if (sdmmc->ipmmu_obj_count >= NUM_IPMMU_OBJECTS) {
        /* When we reached the limit, reuse the last object in tailq,
         * which is considered least used obj */
        obj = TAILQ_LAST(&ipmmu_tailq_head, ipmmu_obj_head);
        /* Need to remap */
        obj = rcar_ipmmu_remap_obj(sdmmc, obj, paddr, len);
        if (obj != NULL) {
            /* Insert at the head of tailq */
            TAILQ_REMOVE(&ipmmu_tailq_head, obj, ipmmu_tailq);
            TAILQ_INSERT_HEAD(&ipmmu_tailq_head, obj, ipmmu_tailq);
        }

    } else {
        /* Create new ipmmu obj */
        if ((obj = calloc(1, sizeof(*obj))) == NULL) {
            return NULL;
        }
        ret = ipmmu_map(sdmmc->ipmmu_handle, paddr, len, &obj->iova);
        if (ret != EOK) {
            free(obj);
            return NULL;
        }
        obj->paddr = paddr;
        obj->len = len;
        /* Insert new ipmmu obj at the head of tailq */
        TAILQ_INSERT_HEAD(&ipmmu_tailq_head, obj, ipmmu_tailq);
        sdmmc->ipmmu_obj_count++;
    }

    return obj;
}

static struct ipmmu_obj * rcar_ipmmu_map_obj(rcar_sdmmc_t *sdmmc, uint64_t paddr, uint32_t len)
{
    struct ipmmu_obj *obj;

    if (!TAILQ_EMPTY(&ipmmu_tailq_head)) {
        /* Lookup similar address + len */
        TAILQ_FOREACH(obj, &ipmmu_tailq_head, ipmmu_tailq) {
            if ((obj->paddr == paddr) && (obj->len == len)) {
                rcar_ipmmu_set_mru_obj(obj);

                /*sdio_slogf(_SLOGC_SDIODI, _SLOG_ERROR, 1, 0,
                    "%s: reuse paddr 0x%lx, len %d", __func__, paddr, len);*/
                return obj;

            } else if ((obj->paddr == paddr) && (obj->len != len)) {
                /* Same address but different length,
                 * reuse ipmmu obj but need to remap first */
                obj = rcar_ipmmu_remap_obj(sdmmc, obj, paddr, len);
                if (obj != NULL) {
                    rcar_ipmmu_set_mru_obj(obj);
                }

                /*sdio_slogf(_SLOGC_SDIODI, _SLOG_ERROR, 1, 0,
                    "%s: remap paddr 0x%lx, len %d", __func__, paddr, len);*/
                return obj;
            }
        }
        /* Not found in tailq, create the new ipmmu obj */
        obj = rcar_ipmmu_create_obj(sdmmc, paddr, len);

        /*sdio_slogf(_SLOGC_SDIODI, _SLOG_ERROR, 1, 0,
            "%s: add paddr 0x%lx, len %d", __func__, paddr, len);*/
        return obj;

    } else {
        /* Create first ipmmu obj */
        obj = rcar_ipmmu_create_obj(sdmmc, paddr, len);

        /*sdio_slogf(_SLOGC_SDIODI, _SLOG_ERROR, 1, 0,
            "%s: first paddr 0x%lx, len %d", __func__, paddr, len);*/
        return obj;
    }

    return NULL;
}

static void rcar_ipmmu_cleanup_obj(rcar_sdmmc_t *sdmmc)
{
    struct ipmmu_obj *obj;

    /* Free the entire tailq */
    while ((obj=TAILQ_FIRST(&ipmmu_tailq_head))) {
        TAILQ_REMOVE(&ipmmu_tailq_head, obj, ipmmu_tailq);
        free(obj);
    }
    sdmmc->ipmmu_obj_count = 0;
    ipmmu_close(sdmmc->ipmmu_handle);
}
#endif /* defined(VARIANT_ipmmu) */

static int rcar_sdmmc_dma_setup(sdio_hc_t * const hc, sdio_cmd_t * const cmd)
{
    rcar_sdmmc_t    *sdmmc;
    sdio_sge_t      *sgp;
    uint32_t        sgc;

    sdmmc = hc->cs_hdl;

    sgc = cmd->sgc;
    sgp = cmd->sgl;

    if (!(cmd->flags & SCF_DATA_PHYS)) {
        sdio_vtop_sg(sgp, sdmmc->sgl, (int)sgc, cmd->mhdl);
        sgp = sdmmc->sgl;
    }

    /* Enable read/write by DMA */
    sdmmc_write(sdmmc->vbase, MMC_CC_EXT_MODE, BUF_ACC_DMAWEN | CC_EXT_MODE_MSK);

    /* Set the address mode */
    sdmmc_write(sdmmc->vbase, MMC_DM_CM_DTRAN_MODE, (uint32_t)(((cmd->flags & SCF_DIR_IN) ? CH_NUM_UPSTREAM : CH_NUM_DOWNSTREAM)
                        | BUS_WID_64BIT | INCREMENT_ADDRESS));

    /* Set the SDMA address */
    if ( (sgp->sg_address & 0x7FULL) ) {
        /* The DMA has an 128 byte alignment requirement */
        sdmmc_write(sdmmc->vbase, MMC_CC_EXT_MODE, CC_EXT_MODE_MSK);

        return( EINVAL );
    }

#if !defined(VARIANT_ipmmu)
    sdmmc_write(sdmmc->vbase, MMC_DM_CM_DTRAN_ADDR, (uint32_t)sgp->sg_address);

#else /* defined(VARIANT_ipmmu) */
    size_t len = cmd->blksz;
    if (cmd->blks > 0) {
        len *= cmd->blks;
    }
    struct ipmmu_obj *obj;
    obj = rcar_ipmmu_map_obj(sdmmc, sgp->sg_address, len);
    if (obj == NULL) {
        sdio_slogf(_SLOGC_SDIODI, _SLOG_ERROR, hc->cfg.verbosity, 0,
        "%s: ipmmu failed to map paddr 0x%lx, len %ld", __func__, sgp->sg_address, len);
        return ENOMEM;
    }
    sdmmc_write(sdmmc->vbase, MMC_DM_CM_DTRAN_ADDR, obj->iova);

#endif /* !defined(VARIANT_ipmmu) */

    sdmmc->flags = 0;

    return (EOK);
}

static void rcar_sdmmc_dma_start( sdio_hc_t * const hc, sdio_cmd_t * const cmd )
{
    rcar_sdmmc_t *sdmmc;

    sdmmc = hc->cs_hdl;

    if (sdmmc->flags & OF_DMA_ACTIVE) {
        return;
    }

    /* Clear DMA status */
    sdmmc_write(sdmmc->vbase, MMC_DM_CM_INFO1, 0);
    sdmmc_write(sdmmc->vbase, MMC_DM_CM_INFO2, 0);

    sdmmc->flags = OF_DMA_ACTIVE;

    /* Enable DMA interrupt */
    if (cmd->flags & SCF_DIR_IN) {
        sdmmc_write(sdmmc->vbase, MMC_DM_CM_INFO1_MASK, sdmmc_read(sdmmc->vbase, MMC_DM_CM_INFO1_MASK) & ~DM_INFO1_DTRAN_END1(sdmmc->dma_tranend1));
    }
    else {
        sdmmc_write(sdmmc->vbase, MMC_DM_CM_INFO1_MASK, sdmmc_read(sdmmc->vbase, MMC_DM_CM_INFO1_MASK) & ~DM_INFO1_DTRAN_END0);
    }

    /* Start DMA */
    sdmmc_write(sdmmc->vbase, MMC_DM_CM_DTRAN_CTRL, DM_START);
}

static void rcar_sdmmc_dma_cmplt( sdio_hc_t * const hc, sdio_cmd_t * const cmd, const int error)
{
    rcar_sdmmc_t  *sdmmc = hc->cs_hdl;

    /* Disable DMA interrupt */
    if (cmd->flags & SCF_DIR_IN) {
        sdmmc_write(sdmmc->vbase, MMC_DM_CM_INFO1_MASK, sdmmc_read(sdmmc->vbase, MMC_DM_CM_INFO1_MASK) | DM_INFO1_DTRAN_END1(sdmmc->dma_tranend1));
    } else {
        sdmmc_write(sdmmc->vbase, MMC_DM_CM_INFO1_MASK, sdmmc_read(sdmmc->vbase, MMC_DM_CM_INFO1_MASK) | DM_INFO1_DTRAN_END0);
    }

    /* Reset DMA channels if error occurs */
    if (error) {
        sdmmc_write(sdmmc->vbase, MMC_DM_CM_RST, DM_RST_REVBITS_MSK & ~(DM_RST_DTRANRST1 | DM_RST_DTRANRST0));
        sdmmc_write(sdmmc->vbase, MMC_DM_CM_RST, DM_RST_REVBITS_MSK |  (DM_RST_DTRANRST1 | DM_RST_DTRANRST0));
    }

    /* Clear DMA Status */
    sdmmc_write(sdmmc->vbase, MMC_DM_CM_INFO1, 0);
    sdmmc_write(sdmmc->vbase, MMC_DM_CM_INFO2, 0);

    /* The SD_BUF read/write DMA transfer is disabled */
    sdmmc_write(sdmmc->vbase, MMC_CC_EXT_MODE, CC_EXT_MODE_MSK);

    sdmmc->flags &= ~OF_DMA_ACTIVE;
}

static int rcar_sdmmc_event(sdio_hc_t *const hc, sdio_event_t * const ev)
{
    const rcar_sdmmc_t    * const sdmmc = hc->cs_hdl;
    int             status = CS_CMD_INPROG;

    switch (ev->code) {
        case HC_EV_INTR:
            status = rcar_sdmmc_intr_event(hc);
            InterruptUnmask(sdmmc->irq, hc->hc_iid);
            break;
        default:
            break;
    }

    return (status);
}

static void rcar_sdmmc_flush_sd_buf(const sdio_hc_t * const hc)
{
    const rcar_sdmmc_t    *sdmmc;

    sdmmc  = hc->cs_hdl;

    if (sdmmc_read(sdmmc->vbase, MMC_SD_INFO2) & SDH_INFO2_BRE) {
        uint32_t i;
        uint32_t mmode;

        mmode = sdmmc_read(sdmmc->vbase, MMC_HOST_MODE);

        /* Change Width for Access to SD_BUF to 32 bit CPU reead */
        sdmmc_write(sdmmc->vbase, MMC_HOST_MODE, mmode | HOST_MODE_BUSWIDTH_32BIT | HOST_MODE_WMODE_N64BIT);

        while (sdmmc_read(sdmmc->vbase, MMC_SD_INFO2) & SDH_INFO2_BRE) {
            sdmmc_write(sdmmc->vbase, MMC_SD_INFO2, sdmmc_read(sdmmc->vbase, MMC_SD_INFO2) & ~SDH_INFO2_BRE); //Clear BRE bit
            for (i = 0; i < sdmmc_read(sdmmc->vbase, MMC_SD_SIZE) / 4; i++) {
                sdmmc_read(sdmmc->vbase, MMC_SD_BUF0); //flush SD_BUF
            }
        }

        /* Recover MMC_HOST_MODE */
        sdmmc_write(sdmmc->vbase, MMC_HOST_MODE, mmode);
    }
}

static int rcar_sdmmc_check_idle(sdio_hc_t * const hc)
{
    const rcar_sdmmc_t    *sdmmc;
    uint32_t        status;
    int             i;

    sdmmc = hc->cs_hdl;

    for (i = 0; i < SDHI_TMOUT; i++) {
        status =  sdmmc_read(sdmmc->vbase, MMC_SD_INFO2);
        if (!(status & SDH_INFO2_CBSY) && (status & SDH_INFO2_SCLKDIVEN)) {
            break;
        }
        rcar_sdmmc_flush_sd_buf(hc);
        nanospin_ns(1000);
    }

    if (i >= SDHI_TMOUT) {
        return (EBUSY);
    }

    return EOK;
}

static int rcar_sdmmc_xfer_setup(sdio_hc_t * const hc, sdio_cmd_t * const cmd)
{
    rcar_sdmmc_t    *sdmmc;
    int             status = EOK;

    sdmmc = hc->cs_hdl;

    if (rcar_sdmmc_check_idle(hc) != EOK) {
        sdio_slogf(_SLOGC_SDIODI, _SLOG_ERROR, hc->cfg.verbosity, 1,
            "%s: ERROR in SDHI%d, BUSY status is present in SD_INFO2 register, unable to make a transfer setup", __func__, sdmmc->chan_idx);
        return (EAGAIN);
    }

    sdio_sg_start(hc, cmd->sgl, cmd->sgc);

    if ((cmd->blksz < 2) || (cmd->blksz > 512)) {
        sdio_slogf(_SLOGC_SDIODI, _SLOG_ERROR, hc->cfg.verbosity, 1,
            "%s: Block size (%d) is not in range of 2 to 512", __func__, cmd->blksz);
        return (EINVAL);
    }

    /* Set block size */
    sdmmc_write(sdmmc->vbase, MMC_SD_SIZE, cmd->blksz);

    /* Set block count (multi-block transfers) */
    if ((cmd->blks > 1) || ((cmd->flags & SCF_MULTIBLK) != 0)) {
        sdmmc_write(sdmmc->vbase, MMC_SD_SECCNT, cmd->blks);
    }

    sdmmc->cmd = cmd;

    if ((cmd->sgc != 0) && (hc->caps & HC_CAP_DMA)) {
        status = rcar_sdmmc_dma_setup(hc, cmd);
        if (status == EOK) {
        }
    }

    return (status);
}

static int rcar_sdmmc_cmd(sdio_hc_t * const hc, sdio_cmd_t * const cmd)
{
    rcar_sdmmc_t  *sdmmc;
    int           status;
    uint32_t      command;

    sdmmc = hc->cs_hdl;

    if (rcar_sdmmc_check_idle(hc) != EOK) {
        sdio_slogf(_SLOGC_SDIODI, _SLOG_ERROR, hc->cfg.verbosity, 0,
            "%s: ERROR in SDHI%d, BUSY status is present in SD_INFO2 register, unable to issue CMD %d", __func__, sdmmc->chan_idx, cmd->opcode);
        return (EAGAIN);
    }

    /* Clear Status */
    sdmmc_write(sdmmc->vbase, MMC_SD_INFO1, sdmmc_read(sdmmc->vbase, MMC_SD_INFO1) & ~(SDH_INFO1_AE | SDH_INFO1_RE));
    sdmmc_write(sdmmc->vbase, MMC_SD_INFO2, 0);

    command = cmd->opcode;
    switch (cmd->flags & (0x1F << 4)) {
        case SCF_RSP_NONE:
            command |= SDH_CMD_NORSP;
            break;
        case SCF_RSP_R1:
            command |= SDH_CMD_RSPR1;
            break;
        case SCF_RSP_R1B:
            command |= SDH_CMD_RSPR1B;
            break;
        case SCF_RSP_R2:
            command |= SDH_CMD_RSPR2;
            break;
        case SCF_RSP_R3:
            command |= SDH_CMD_RSPR3;
            break;
        default:
            break;
    }

    sdmmc->flags = 0;

    if ((cmd->flags & SCF_CTYPE_ADTC) != 0) {
        command |= SDH_CMD_ADTC;
        if (cmd->flags & SCF_DIR_IN) {
            command |= SDH_CMD_DAT_READ;
        }
        if ((cmd->flags & SCF_MULTIBLK) != 0) {
              sdmmc_write(sdmmc->vbase, MMC_SD_STOP, SDH_STOP_SEC);
            command |= SDH_CMD_DAT_MULTI;
            if ((hc->caps & HC_CAP_ACMD12) == 0) {
                command |= SDH_CMD_NOAC12;
            }
        } else {
            sdmmc_write(sdmmc->vbase, MMC_SD_STOP, 0);
        }

        status = rcar_sdmmc_xfer_setup(hc, cmd);
        if (status != EOK) {
            return (status);
        }

        /* card insertion/removal are always enabled */
        sdmmc_write(sdmmc->vbase, MMC_SD_INFO1_MASK, (uint32_t)(~(SDH_INFO1_AE | SDH_INFO1_RE | SDH_INFO1_RMVL | SDH_INFO1_INST)));
    } else {
        sdmmc_write(sdmmc->vbase, MMC_SD_INFO1_MASK, (uint32_t)(~(SDH_INFO1_RE | SDH_INFO1_RMVL | SDH_INFO1_INST)));
    }

    sdmmc_write(sdmmc->vbase, MMC_SD_INFO2_MASK, (uint32_t)(~SDH_INFO2_ALL_ERR));

    sdmmc_write(sdmmc->vbase, MMC_SD_ARG, cmd->arg);

    sdmmc_write(sdmmc->vbase, MMC_SD_CMD, command);

    return (EOK);
}

static int rcar_sdmmc_abort(sdio_hc_t * const hc, sdio_cmd_t * const cmd)
{
    const rcar_sdmmc_t *sdmmc;

    sdmmc = hc->cs_hdl;

    if ((cmd->flags & SCF_CTYPE_ADTC) != 0) {
        sdmmc_write(sdmmc->vbase, MMC_SD_STOP, SDH_STOP_STP);
    }

    if ((sdmmc->flags & OF_DMA_ACTIVE) != 0) {
        rcar_sdmmc_dma_cmplt(hc, cmd, 1);
    }

    return (EOK);
}

static int rcar_sdmmc_pwr(sdio_hc_t * hc, const uint32_t vdd)
{
    rcar_sdmmc_t *sdmmc;
    int timeout = 1000;

    sdmmc = hc->cs_hdl;

    if (vdd == 0) {
        sdmmc_write(sdmmc->vbase, MMC_SOFT_RST, sdmmc_read(sdmmc->vbase, MMC_SOFT_RST) & ~SOFT_RST_OFF);
        delay(1);
        sdmmc_write(sdmmc->vbase, MMC_SOFT_RST, sdmmc_read(sdmmc->vbase, MMC_SOFT_RST) | SOFT_RST_OFF);
        /* Check reset release state */
        while(((sdmmc_read(sdmmc->vbase, MMC_SOFT_RST) & SOFT_RST_OFF) == 0) && (timeout > 0)) {
            nanospin_ns(1000);
            timeout--;
        }

        /* Reset DMA channels */
        sdmmc_write(sdmmc->vbase, MMC_DM_CM_RST, DM_RST_REVBITS_MSK & ~(DM_RST_DTRANRST1 | DM_RST_DTRANRST0));
        sdmmc_write(sdmmc->vbase, MMC_DM_CM_RST, DM_RST_REVBITS_MSK |  (DM_RST_DTRANRST1 | DM_RST_DTRANRST0));

        /* Stop clock */
        sdmmc_write(sdmmc->vbase, MMC_SD_CLK_CTRL, sdmmc_read(sdmmc->vbase, MMC_SD_CLK_CTRL) & ~SDH_CLKCTRL_SCLKEN);

        /* Disable SCC */
        sdmmc_write(sdmmc->vbase, MMC_SCC_CKSEL, ~MMC_SCC_CKSEL_DTSEL & sdmmc_read(sdmmc->vbase, MMC_SCC_CKSEL));
        sdmmc_write(sdmmc->vbase, MMC_SCC_DTCNTL, ~MMC_SCC_DTCNTL_TAPEN & sdmmc_read(sdmmc->vbase, MMC_SCC_DTCNTL));
        sdmmc_write(sdmmc->vbase, MMC_SCC_RVSCNTL, ~MMC_SCC_RVSCNTL_RVSEN & sdmmc_read(sdmmc->vbase, MMC_SCC_RVSCNTL));
        sdmmc_write(sdmmc->vbase, MMC_SCC_DT2FF, sdmmc->tap);

        if ((hc->flags & HC_FLAG_DEV_MMC) != 0) {
            rcar_sdmmc_scc_hs400_reset(hc);
        }

        /* Start clock */
        sdmmc_write(sdmmc->vbase, MMC_SD_CLK_CTRL, sdmmc_read(sdmmc->vbase, MMC_SD_CLK_CTRL) | SDH_CLKCTRL_SCLKEN);

        sdmmc->tuning_status = RCAR_TUNING_NONE;
    }

    hc->vdd = vdd;

    return (EOK);
}

static int rcar_sdmmc_bus_mode(sdio_hc_t * hc, const uint32_t bus_mode)
{
    hc->bus_mode = bus_mode;

    return (EOK);
}

static int rcar_sdmmc_bus_width(sdio_hc_t * hc, const uint32_t width)
{
    const rcar_sdmmc_t *sdmmc;
    uint32_t    hctl;

    sdmmc = hc->cs_hdl;

    if (rcar_sdmmc_check_idle(hc) != EOK) {
        sdio_slogf(_SLOGC_SDIODI, _SLOG_ERROR, hc->cfg.verbosity, 0,
            "%s: ERROR in SDHI%d, BUSY status is present in SD_INFO2 register, unable to set bus width", __func__, sdmmc->chan_idx);
        return (EAGAIN);
    }

    hctl = sdmmc_read(sdmmc->vbase, MMC_SD_OPTION) & ~(SDH_OPTION_WIDTH_1 | SDH_OPTION_WIDTH_8);

    if (width == 8) {
        hctl |=  (SDH_OPTION_WIDTH_8);
    }
    else if (width == 1) {
        hctl |=  (SDH_OPTION_WIDTH_1);
    }
    else {
        // nothing
    }

    sdmmc_write(sdmmc->vbase, MMC_SD_OPTION, hctl);

    hc->bus_width = width;

    return (EOK);
}

static uint8_t clock_div(const uint32_t hclk, uint32_t *clk, const uint32_t timing)
{
    uint32_t clk_t;
    uint32_t new_clock;

    if (*clk >= hclk) {
        /*  The output clock is the 1/2 frequency mode */
        if (timing == TIMING_HS400) {
            clk_t = 0;
        } else {
            clk_t = 0xFF;
        }

        *clk = hclk;
    }
    else {
        clk_t = 0x80;
        for (new_clock = hclk/512; *clk >= (new_clock * 2); ) {
            new_clock <<= 1;
            clk_t >>= 1;
        }

        *clk = new_clock;
    }

    return (uint8_t)clk_t;
}

static int rcar_sdmmc_clk(sdio_hc_t *hc, const uint32_t clk)
{
    rcar_sdmmc_t  *sdmmc;
    uint32_t      clkctl;
    uint32_t      clk_t = clk;
    uint32_t      pclock;

    sdmmc = hc->cs_hdl;

    if (rcar_sdmmc_check_idle(hc) != EOK) {
        sdio_slogf(_SLOGC_SDIODI, _SLOG_ERROR, hc->cfg.verbosity, 0,
            "%s: ERROR in SDHI%d, BUSY status is present in SD_INFO2 register, unable to change clock", __func__, sdmmc->chan_idx);
        return (EAGAIN);
    }

    if (clk_t > hc->clk_max) {
        clk_t = hc->clk_max;
    }

    /*
     * Both HS400 and HS200/SD104 set 200MHz, but some devices need to
     * set 400MHz to distinguish the CPG SDnCLCR settings in HS400.
     * 8 TAP: SRCFC=1, FC=0
     * 4 TAP: SRCFC=0, FC=1
     */
    if ((hc->timing == TIMING_HS400) && (sdmmc->hs400_use_4tap != 0) && (clk_t == 200000000)) {
        clk_t = 400000000;
    }

    /*
     * In R-Car SDHI, internal SCC modules uses SDnH clock.
     * It is controlled by SDnCKCR.STPnHCK bit in CPG.
     * When SD clock is less than High Speed, SDnH clock is stopped.
     * And SDnH clock is supplied with 100MHz or more in Clock divider
     * table of CPG in R-Car.
     * It is the recommended setting of H/W.
     *
     * In SDR104/HS200/HS400 mode, SDnH clock must supply for SCC
     */
    if ( ((hc->timing == TIMING_SDR104) || (hc->timing == TIMING_HS200) || (hc->timing == TIMING_HS400)) &&
            (clk_t < sdmmc->scc_min_clk) ) {
        clk_t = sdmmc->scc_min_clk;
    }

    /* Stop clock */
    sdmmc_write(sdmmc->vbase, MMC_SD_CLK_CTRL, sdmmc_read(sdmmc->vbase, MMC_SD_CLK_CTRL) & ~SDH_CLKCTRL_SCLKEN);

    pclock = clk_t;

#ifdef RCAR_CPG
    if (cpg_hwfuncs.clk_rate_set(CPG_CLK_SDHI, sdmmc->chan_idx, &pclock)) {
        sdio_slogf(_SLOGC_SDIODI, _SLOG_INFO, hc->cfg.verbosity, 1, "Cannot change SDnCLCR register\n");
        pclock = sdmmc->pclk;
    }
#endif

    clkctl  = sdmmc_read(sdmmc->vbase, MMC_SD_CLK_CTRL);
    clkctl &= 0xFFFFFF00;
    clkctl |= clock_div(pclock, &clk_t, hc->timing);

    sdmmc_write(sdmmc->vbase, MMC_SD_CLK_CTRL, clkctl);

     /* Start clock */
    sdmmc_write(sdmmc->vbase, MMC_SD_CLK_CTRL, sdmmc_read(sdmmc->vbase, MMC_SD_CLK_CTRL) | SDH_CLKCTRL_SCLKEN);

    hc->clk = clk_t;
    sdmmc->busclk = clk_t;

    return (EOK);
}

static int rcar_sdmmc_signal_voltage(sdio_hc_t * const hc, const uint32_t signal_voltage)
{
    return (EOK);
}

static int rcar_sdmmc_timing(sdio_hc_t * hc, const uint32_t timing)
{
    rcar_sdmmc_t  *sdmmc;
    uint32_t      new_tap;

    sdmmc = hc->cs_hdl;

    hc->timing = timing;

    rcar_sdmmc_clk(hc, hc->clk);

    if (rcar_sdmmc_check_idle(hc) != EOK) {
        sdio_slogf(_SLOGC_SDIODI, _SLOG_ERROR, hc->cfg.verbosity, 1,
            "%s: ERROR in SDHI%d, BUSY status is present in SD_INFO2 register, unable to change timing", __func__, sdmmc->chan_idx);
        return (EAGAIN);
    }

    /* Stop clock */
    sdmmc_write(sdmmc->vbase, MMC_SD_CLK_CTRL, sdmmc_read(sdmmc->vbase, MMC_SD_CLK_CTRL) & ~SDH_CLKCTRL_SCLKEN);

    if (timing == TIMING_HS400 ) {
        /* Set HS400 mode */
        sdmmc_write(sdmmc->vbase, MMC_SDIF_MODE, sdmmc_read(sdmmc->vbase, MMC_SDIF_MODE) | SDIF_MODE_HS400);

        sdmmc_write(sdmmc->vbase, MMC_SCC_DT2FF, sdmmc->tap_hs400);

        if (sdmmc->hs400_manual_correction != 0) {
            /* Disable SCC sampling clock position correction */
            sdmmc_write(sdmmc->vbase, MMC_SCC_RVSCNTL, ~MMC_SCC_RVSCNTL_RVSEN &
                sdmmc_read(sdmmc->vbase, MMC_SCC_RVSCNTL));
        }

        sdmmc_write(sdmmc->vbase, MMC_SCC_TMPPORT2,  sdmmc_read(sdmmc->vbase, MMC_SCC_TMPPORT2) |
                MMC_SCC_TMPPORT2_HS400OSEL | MMC_SCC_TMPPORT2_HS400EN);

        sdmmc_write(sdmmc->vbase, MMC_SCC_DTCNTL, MMC_SCC_DTCNTL_TAPEN | (0x4 << 16));

        /* Avoid bad TAP */
        if (sdmmc->hs400_bad_tap & (1 << sdmmc->tap_set)) {
            new_tap = (sdmmc->tap_set + sdmmc->tap_num + 1) % sdmmc->tap_num;

            if (sdmmc->hs400_bad_tap & (1 << new_tap)) {
                new_tap = (sdmmc->tap_set + sdmmc->tap_num - 1) % sdmmc->tap_num;
            }

            if (sdmmc->hs400_bad_tap & (1 << new_tap)) {
                sdio_slogf(_SLOGC_SDIODI, _SLOG_INFO, hc->cfg.verbosity, 0,
                    "%s: Three consecutive bad tap is prohibited, TAPSET cannot change from %d to %d", __func__, sdmmc->tap_set, new_tap);
                new_tap = sdmmc->tap_set;
            }

            sdmmc->tap_set = new_tap;
        }

        /* Replace the tuning result of 8TAP with 4TAP */
        if ((sdmmc->hs400_use_4tap) != 0) {
            sdmmc_write(sdmmc->vbase, MMC_SCC_TAPSET, sdmmc->tap_set / 2);
        } else {
            sdmmc_write(sdmmc->vbase, MMC_SCC_TAPSET, sdmmc->tap_set);
        }

        sdmmc_write(sdmmc->vbase, MMC_SCC_CKSEL, sdmmc_read(sdmmc->vbase, MMC_SCC_CKSEL) | MMC_SCC_CKSEL_DTSEL);

        /* Apply HS400 SW work around */
        if ((sdmmc->adjust_hs400_enable) != 0) {
            /* Set this flag to execute adjust hs400 offset after setting to HS400 mode and after CMD13 */
            sdmmc->need_adjust_hs400 = 1;
        }
    } else {
        rcar_sdmmc_disable_scc(hc);

        /* Reset HS400 mode */
        if ((hc->flags & HC_FLAG_DEV_MMC) != 0) {
            rcar_sdmmc_scc_hs400_reset(hc);
        }
    }

    /* Start clock */
    sdmmc_write(sdmmc->vbase, MMC_SD_CLK_CTRL, sdmmc_read(sdmmc->vbase, MMC_SD_CLK_CTRL) | SDH_CLKCTRL_SCLKEN);

    if (timing == TIMING_HS400 ) {
        rcar_sdmmc_debug(hc, "HS400 Tuning");
    }

    return (EOK);
}

static int rcar_sdmmc_select_tuning(sdio_hc_t * const hc)
{
    rcar_sdmmc_t *sdmmc;
    uint32_t      tapcnt;    /* Count 'OK' TAP */
    int           tapset;    /* Tap position */
    uint32_t      tapstart;  /* Start position of OK' TAP */
    uint32_t      tapend;    /* End position of OK' TAP */
    uint32_t      ntap;      /* Number of 'OK' TAP */
    uint32_t      matchcnt;  /* Count of matching data */
    uint32_t      idx;

    sdmmc = hc->cs_hdl;

    /* Clear adjust hs400 flag */
    sdmmc->need_adjust_hs400 = 0;
    sdmmc->tuning_status = RCAR_TUNING_DONE;

    /* Clear SCC_RVSREQ */
    sdmmc_write(sdmmc->vbase, MMC_SCC_RVSREQ, 0);

    /* If TAP is 'NG', force its repeated display index to 'NG' */
    for (idx= 0; idx < sdmmc->tap_num * 2; idx++) {
        if (!sdmmc->taps[idx]) {
            sdmmc->taps[idx % sdmmc->tap_num] = 0;
            sdmmc->taps[(idx % sdmmc->tap_num) + sdmmc->tap_num] = 0;
        }
        if (!sdmmc->smpcmp[idx]) {
            sdmmc->smpcmp[idx % sdmmc->tap_num] = 0;
            sdmmc->smpcmp[(idx % sdmmc->tap_num) + sdmmc->tap_num] = 0;
        }
    }

    /* Find largest range of TAPs are 'OK'. The sampling clock position = (start position + end position) / 2 */
    tapset   = -1;
    tapcnt   = 0;
    ntap     = 0;
    tapstart = 0;
    tapend   = 0;
    for (idx= 0; idx< sdmmc->tap_num * 2; idx++) {
        if (sdmmc->taps[idx]) {
            ntap++;
        } else {
            if (ntap > tapcnt) {
                tapstart = idx- ntap;
                tapend   = idx- 1;
                tapcnt   = ntap;
            }
            ntap = 0;
        }
    }

    if (ntap > tapcnt) {
        tapstart = idx- ntap;
        tapend   = idx- 1;
        tapcnt   = ntap;
    }

    if ((tapcnt < sdmmc->tap_num * 2) && (tapcnt >= RCAR_SDHI_MAX_TAP)) {
        tapset = (int)(((tapstart + tapend) / 2) % sdmmc->tap_num);
    }
    else if (tapcnt == sdmmc->tap_num * 2) {
        /* If all TAP is 'OK', the sampling clock position is selected by
         * identifying the change point of data */
        matchcnt = 0;
        ntap     = 0;
        tapstart = 0;
        tapend   = 0;
        for (idx= 0; idx< sdmmc->tap_num * 2; idx++) {
            if (sdmmc->smpcmp[idx]) {
                ntap++;
            } else {
                if (ntap > matchcnt) {
                    tapstart = idx- ntap;
                    tapend   = idx- 1;
                    matchcnt = ntap;
                }
                ntap = 0;
            }
        }
        if (ntap > matchcnt) {
            tapstart = idx- ntap;
            tapend   = idx- 1;
            matchcnt = ntap;
        }
        if (matchcnt != 0) {
            tapset = (int)(((tapstart + tapend) / 2) % sdmmc->tap_num);
        }
    }
    else {
        // nothing
    }

    /* Tuning is failed */
    if (tapset == -1) {
        return (EIO);
    }

    /* Set TAPSET */
    sdmmc_write(sdmmc->vbase, MMC_SCC_TAPSET, (uint32_t)tapset);

    /* Save for HS400 case */
    sdmmc->tap_set = (uint32_t)tapset;

    /* Enable auto-retuning */
    sdmmc_write(sdmmc->vbase, MMC_SCC_RVSCNTL, MMC_SCC_RVSCNTL_RVSEN | sdmmc_read(sdmmc->vbase, MMC_SCC_RVSCNTL));

    rcar_sdmmc_debug(hc, "Select HS200 Tuning");

    return (EOK);
}

static void rcar_sdmmc_scc_reset(sdio_hc_t * const hc)
{
    rcar_sdmmc_t *sdmmc;

    sdmmc = hc->cs_hdl;

    /* Stop clock */
    sdmmc_write(sdmmc->vbase, MMC_SD_CLK_CTRL, sdmmc_read(sdmmc->vbase, MMC_SD_CLK_CTRL) & ~SDH_CLKCTRL_SCLKEN);

    /* Reset SCC */
    sdmmc_write(sdmmc->vbase, MMC_SCC_CKSEL, ~MMC_SCC_CKSEL_DTSEL & sdmmc_read(sdmmc->vbase, MMC_SCC_CKSEL));
    sdmmc_write(sdmmc->vbase, MMC_SCC_DTCNTL, ~MMC_SCC_DTCNTL_TAPEN & sdmmc_read(sdmmc->vbase, MMC_SCC_DTCNTL));

    if ((hc->flags & HC_FLAG_DEV_MMC) != 0) {
        rcar_sdmmc_scc_hs400_reset(hc);
    }

    /* Start clock */
    sdmmc_write(sdmmc->vbase, MMC_SD_CLK_CTRL, sdmmc_read(sdmmc->vbase, MMC_SD_CLK_CTRL) | SDH_CLKCTRL_SCLKEN);

    sdmmc_write(sdmmc->vbase, MMC_SCC_RVSCNTL, ~MMC_SCC_RVSCNTL_RVSEN & sdmmc_read(sdmmc->vbase, MMC_SCC_RVSCNTL));
    sdmmc_write(sdmmc->vbase, MMC_SCC_RVSCNTL, ~MMC_SCC_RVSCNTL_RVSEN & sdmmc_read(sdmmc->vbase, MMC_SCC_RVSCNTL));

    sdmmc->tuning_status = RCAR_TUNING_NONE;
}

static void rcar_sdmmc_disable_scc(sdio_hc_t * const hc)
{
    const rcar_sdmmc_t *sdmmc;

    sdmmc = hc->cs_hdl;

    if ((hc->timing == TIMING_SDR104) || (hc->timing == TIMING_HS200) || (hc->timing == TIMING_HS400)) {
        return;
    }

    sdmmc_write(sdmmc->vbase, MMC_SCC_CKSEL, ~MMC_SCC_CKSEL_DTSEL & sdmmc_read(sdmmc->vbase, MMC_SCC_CKSEL));
    sdmmc_write(sdmmc->vbase, MMC_SCC_DTCNTL, ~MMC_SCC_DTCNTL_TAPEN & sdmmc_read(sdmmc->vbase, MMC_SCC_DTCNTL));
}


static int rcar_sdmmc_tune(sdio_hc_t *hc, const uint32_t op)
{
    rcar_sdmmc_t    *sdmmc;
    struct sdio_cmd *cmd;
    sdio_sge_t      sge;
    uint8_t         *td;
    const uint8_t   *sdio_tbp;
    uint32_t        tlen;
    int             status;
    uint32_t        idx;
    uint32_t        tapnum;

    /* return if not HS200 or SDR104 that requires tuning */
    if ((hc->timing != TIMING_SDR104) && (hc->timing != TIMING_HS200)) {
        return (EOK);
    }

    sdmmc = hc->cs_hdl;

    tapnum  = 8;
    if (hc->bus_width == 8) {
        tlen = 128;
        sdio_tbp = sdio_tbp_8bit;
    }
    else {
        tlen = 64;
        sdio_tbp = sdio_tbp_4bit;
    }

    /* Buffer of tuning command */
    cmd = sdio_alloc_cmd();
    if (cmd == NULL) {
        return (ENOMEM);
    }

    /* Buffer for reading tuning data */
    td = sdio_alloc(tlen);
    if (td == NULL) {
        sdio_free_cmd(cmd);
        return (ENOMEM);
    }

    sdmmc->tuning_status = RCAR_TUNING_DOING;

    memset(sdmmc->taps, 0, RCAR_SDHI_TUNING_RETRIES);   // Initialize all tap  to 'NG'
    memset(sdmmc->smpcmp, 0, RCAR_SDHI_TUNING_RETRIES); // Initialize all smpcmp to 'NG'

    /* Clear SD flag status */
    sdmmc_write(sdmmc->vbase, MMC_SD_INFO1, 0);
    sdmmc_write(sdmmc->vbase, MMC_SD_INFO2, 0);

    /* Stop clock */
    sdmmc_write(sdmmc->vbase, MMC_SD_CLK_CTRL, sdmmc_read(sdmmc->vbase, MMC_SD_CLK_CTRL) & ~SDH_CLKCTRL_SCLKEN);

    /* Initialize SCC */
    sdmmc_write(sdmmc->vbase, MMC_SCC_DTCNTL, MMC_SCC_DTCNTL_TAPEN | (tapnum << 16));
    sdmmc_write(sdmmc->vbase, MMC_SCC_RVSCNTL, ~MMC_SCC_RVSCNTL_RVSEN & sdmmc_read(sdmmc->vbase, MMC_SCC_RVSCNTL));
    sdmmc_write(sdmmc->vbase, MMC_SCC_DT2FF, sdmmc->tap);
    sdmmc_write(sdmmc->vbase, MMC_SCC_CKSEL, MMC_SCC_CKSEL_DTSEL | sdmmc_read(sdmmc->vbase, MMC_SCC_CKSEL));

    /* Start clock */
    sdmmc_write(sdmmc->vbase, MMC_SD_CLK_CTRL, sdmmc_read(sdmmc->vbase, MMC_SD_CLK_CTRL) | SDH_CLKCTRL_SCLKEN);

    rcar_sdmmc_debug(hc, "HS200 tuning");

    sdmmc->tap_num = (sdmmc_read(sdmmc->vbase, MMC_SCC_DTCNTL) >> 16) & 0xFF;

    /* Issue CMD19/CMD21, 2 times for each tap */
    for (idx = 0; ((idx < sdmmc->tap_num * 2) && (idx < RCAR_SDHI_TUNING_RETRIES)); idx++) {
        /* Clear tuning data buffer to avoid comparing old data after each unsuccessful transfer */
        memset(td, 0, tlen);
        memset(cmd, 0, sizeof( struct sdio_cmd));

        /* Change the sampling clock position */
        sdmmc_write(sdmmc->vbase, MMC_SCC_TAPSET, idx % sdmmc->tap_num);

        /* Setup tuning command */
        sdio_setup_cmd(cmd, SCF_CTYPE_ADTC | SCF_RSP_R1, op, 0);
        sge.sg_count	= tlen;
        sge.sg_address	= (paddr_t)td;
        sdio_setup_cmd_io(cmd, SCF_DIR_IN, 1, tlen, &sge, 1, NULL);

        /* Execute and wait for tuning command completed */
        sdio_issue_cmd(&hc->device, cmd, RCAR_SDHI_TUNING_TIMEOUT);

        /* Tuning command is completed successfully, compare result data with tuning block pattern.
         * It is to determine the sampling clock position */
        if ((cmd->status == CS_CMD_CMP)) {
            if (!(memcmp(td, sdio_tbp, tlen))) {
                sdmmc->taps[idx] = 1; //This TAP is 'OK'
            }
        } else { /* Fix tuning error on some hardware platforms */
            /* A delay of 3.75ms (=150ms/40) based on the specification that the device should complete 40 commands in 150msec */
            delay(4);
        }

        if (!sdmmc_read(sdmmc->vbase, MMC_SCC_SMPCMP)) {
            sdmmc->smpcmp[idx] = 1; //smpcmp of this TAP is 'OK'
        }
    }

    status = rcar_sdmmc_select_tuning(hc);

    sdio_free(td, tlen);
    sdio_free_cmd(cmd);

    if (status != EOK) {
        rcar_sdmmc_scc_reset(hc);
    }

    return (status);
}

static uint32_t sdmmc_tmpport_read(const uintptr_t base, const uint32_t addr)
{
    /* Read mode */
    sdmmc_write(base, MMC_SCC_TMPPORT5, MMC_SCC_TMPPORT5_DLL_RW_SEL_R |
               (MMC_SCC_TMPPORT5_DLL_ADR_MASK & addr));

    /* Access start and stop */
    sdmmc_write(base, MMC_SCC_TMPPORT4, MMC_SCC_TMPPORT4_DLL_ACC_START);
    sdmmc_write(base, MMC_SCC_TMPPORT4, 0);

    /* Read and return value */
    return sdmmc_read(base, MMC_SCC_TMPPORT7);
}

static void sdmmc_tmpport_write(const uintptr_t base, const uint32_t addr, const uint32_t val)
{
    /* Write mode */
    sdmmc_write(base, MMC_SCC_TMPPORT5, MMC_SCC_TMPPORT5_DLL_RW_SEL_W |
               (MMC_SCC_TMPPORT5_DLL_ADR_MASK & addr));

    /* Write value */
    sdmmc_write(base, MMC_SCC_TMPPORT6, val);

    /* Access start and stop */
    sdmmc_write(base, MMC_SCC_TMPPORT4, MMC_SCC_TMPPORT4_DLL_ACC_START);
    sdmmc_write(base, MMC_SCC_TMPPORT4, 0);
}

static void rcar_sdmmc_hs400_adjust_enable(sdio_hc_t * const hc)
{
    rcar_sdmmc_t    *sdmmc;
    uint32_t        calib_code;

    sdmmc = hc->cs_hdl;

    if (sdmmc->adjust_hs400_calib_table == 0) {
        return;
    }

    /* Release WriteProtect of DLL built-in register:
     *  Instruct Write: Target register :MD(H’00)
     *  Set WriteProtect release code(H’A500_0000)
     */
    sdmmc_tmpport_write(sdmmc->vbase, 0x00, MMC_SCC_TMPPORT_DISABLE_WP_CODE);

    /* Reading the calibration code value:
     *  Instruction of Read: Target register :DQSRMONH0(H’26)
     */
    calib_code = sdmmc_tmpport_read(sdmmc->vbase, 0x26);
    calib_code &= MMC_SCC_TMPPORT_CALIB_CODE_MASK;
    calib_code = sdmmc->adjust_hs400_calib_table[calib_code];

    /* Adjust internal DS signal:
     *  Instruct Write: Target register :DQSRSETH0(H’22)
     *  Set DQSRSETH0.DQS00RSET(m) and write calibration code
     *  Set DelayLine Offset value(n) to TMPPORT3
     */
    sdmmc_tmpport_write(sdmmc->vbase, 0x22, MMC_SCC_TMPPORT_MANUAL_MODE | calib_code);
    sdmmc_write(sdmmc->vbase, MMC_SCC_TMPPORT3, sdmmc->adjust_hs400_offset);

    sdmmc->need_adjust_hs400_done = 1;

    /* Clear adjust HS400 mode flag */
    sdmmc->need_adjust_hs400 = 0;

    rcar_sdmmc_debug(hc, "HS400 Adjust Enable");
}

static void rcar_sdmmc_hs400_adjust_disable(sdio_hc_t * const hc)
{
    rcar_sdmmc_t    *sdmmc;

    sdmmc = hc->cs_hdl;

    /* Release WriteProtect of DLL built-in register:
     *  Instruct Write: Target register :MD(H’00)
     *  Set WriteProtect release code(H’A500_0000)
     */
    sdmmc_tmpport_write(sdmmc->vbase, 0x00, MMC_SCC_TMPPORT_DISABLE_WP_CODE);

    /* Releases the adjustment of the internal DS signal
     *  Disabled Manual Calibration W(addr=0x22, 0)
     *  Clear offset value to TMPPORT3
     */
    sdmmc_tmpport_write(sdmmc->vbase, 0x22, 0);
    sdmmc_write(sdmmc->vbase, MMC_SCC_TMPPORT3, 0);

    sdmmc->need_adjust_hs400_done = 0;
}

static void rcar_sdmmc_scc_hs400_reset(sdio_hc_t * const hc)
{
    const rcar_sdmmc_t *sdmmc;

    sdmmc = hc->cs_hdl;

    /* Reset HS4000 mode */
    sdmmc_write(sdmmc->vbase, MMC_SDIF_MODE, sdmmc_read(sdmmc->vbase, MMC_SDIF_MODE) & ~SDIF_MODE_HS400);
    sdmmc_write(sdmmc->vbase, MMC_SCC_DT2FF, sdmmc->tap);
    sdmmc_write(sdmmc->vbase, MMC_SCC_TMPPORT2,  sdmmc_read(sdmmc->vbase, MMC_SCC_TMPPORT2) &
            ~(MMC_SCC_TMPPORT2_HS400OSEL | MMC_SCC_TMPPORT2_HS400EN));

    if (sdmmc->need_adjust_hs400_done == 1) {
        rcar_sdmmc_hs400_adjust_disable(hc);
    }
}

static int rcar_sdmmc_scc_manual_correction(sdio_hc_t * const hc)
{
    rcar_sdmmc_t  *sdmmc = hc->cs_hdl;
    uint32_t      rvsreq;
    uint32_t      smpcmp;
    uint32_t      new_tap = sdmmc->tap_set;
    uint32_t      err_tap = sdmmc->tap_set;

    rvsreq = sdmmc_read(sdmmc->vbase, MMC_SCC_RVSREQ);

    /* No error */
    if (!rvsreq) {
        return (EOK);
    }

    sdmmc_write(sdmmc->vbase, MMC_SCC_RVSREQ, 0);

    /* Change TAP position according to correction status */
    if ((sdmmc->hs400_ignore_dat_correction != 0) && (hc->timing == TIMING_HS400)) {
        /*
         * Correction Error Status contains CMD and DAT signal status.
         * In HS400, DAT signal based on DS signal, not CLK.
         * Therefore, use only CMD status.
         */
        smpcmp = sdmmc_read(sdmmc->vbase, MMC_SCC_SMPCMP) & MMC_SCC_SMPCMP_CMD_ERR;

        switch (smpcmp) {
        case 0:
            return EOK;   /* No error in CMD signal */
        case MMC_SCC_SMPCMP_CMD_REQUP:
            new_tap = (sdmmc->tap_set + sdmmc->tap_num + 1) % sdmmc->tap_num;
            err_tap = (sdmmc->tap_set + sdmmc->tap_num - 1) % sdmmc->tap_num;
            break;
        case MMC_SCC_SMPCMP_CMD_REQDOWN:
            new_tap = (sdmmc->tap_set + sdmmc->tap_num - 1) % sdmmc->tap_num;
            err_tap = (sdmmc->tap_set + sdmmc->tap_num + 1) % sdmmc->tap_num;
            break;
        default:
            sdio_slogf(_SLOGC_SDIODI, _SLOG_ERROR, hc->cfg.verbosity, 0,
                "%s: ERROR in SCC, SCC_RVSREQ 0x%08X, SCC_SMPCMP 0x%08X, Need retune",
                __func__, rvsreq, smpcmp);
            return -(EIO);   /* Need re-tune */
        }

        if (sdmmc->hs400_bad_tap & (1 << new_tap)) {
            /* New tap is bad tap (cannot change).
               Compare  with HS200 tunning result, if smpcmp[err_tap] is OK, perform retune */

            if (sdmmc->smpcmp[err_tap]) {
                return -(EIO);   /* Need re-tune */
            }

            return (EOK);   /* cannot change */
        }

        sdmmc->tap_set = new_tap;
    } else {
        if ((rvsreq & MMC_SCC_RVSREQ_RVSERR) != 0) {
            sdio_slogf(_SLOGC_SDIODI, _SLOG_ERROR, hc->cfg.verbosity, 0,
                "%s: ERROR in SCC, SCC_RVSREQ 0x%08X, Need retune", __func__, rvsreq);
            return -(EIO);    /* Need re-tune */
        } else if ((rvsreq & MMC_SCC_RVSREQ_REQTAPUP) != 0) {
            sdmmc->tap_set = (sdmmc->tap_set + sdmmc->tap_num + 1) % sdmmc->tap_num;
        } else if ((rvsreq & MMC_SCC_RVSREQ_REQTAPDWN) != 0) {
            sdmmc->tap_set = (sdmmc->tap_set + sdmmc->tap_num - 1) % sdmmc->tap_num;
        } else {
            return (EOK);
        }
    }

    /* Set TAP position */
    if ((sdmmc->hs400_use_4tap) != 0) {
        sdmmc_write(sdmmc->vbase, MMC_SCC_TAPSET, sdmmc->tap_set / 2);
    } else {
        sdmmc_write(sdmmc->vbase, MMC_SCC_TAPSET, sdmmc->tap_set);
    }

    return (EOK);
}

static int rcar_sdmmc_scc_auto_correction(sdio_hc_t * const hc)
{
    const rcar_sdmmc_t * const sdmmc = hc->cs_hdl;
    uint32_t     rvsreq;

    /* Check SCC error */
    rvsreq = sdmmc_read(sdmmc->vbase, MMC_SCC_RVSREQ);
    if ( (rvsreq & MMC_SCC_RVSREQ_RVSERR) != 0) {

        sdio_slogf(_SLOGC_SDIODI, _SLOG_ERROR, hc->cfg.verbosity, 1,
            "%s: ERROR in SCC, SCC_RVSREQ 0x%08X, Need retune", __func__, rvsreq);

        /* Clear SCC error */
        sdmmc_write(sdmmc->vbase, MMC_SCC_RVSREQ, 0);

        return -(EIO); /* Need re-tune */
    }

    return (EOK); /* No error */
}

static int rcar_sdmmc_scc_error_check(sdio_hc_t * const hc, const uint32_t info2)
{
    const rcar_sdmmc_t *sdmmc;

    sdmmc = hc->cs_hdl;

    if ((hc->timing != TIMING_SDR104) &&
        (hc->timing != TIMING_HS200)  &&
        ((hc->timing != TIMING_HS400) && (sdmmc->hs400_use_4tap == 0))) {
        return (EOK);
    }

    if ((sdmmc->tuning_status == RCAR_TUNING_DOING )||
        (sdmmc->tuning_status == RCAR_NEED_RETUNING)) {
        return (EOK);
    }

    if ((info2 & (SDH_INFO2_CMDE | SDH_INFO2_CRCE | SDH_INFO2_ENDE | SDH_INFO2_DTO | SDH_INFO2_RTO)) != 0) {
        /* Clear SCC error */
        sdmmc_write(sdmmc->vbase, MMC_SCC_RVSREQ, 0);

        sdio_slogf(_SLOGC_SDIODI, _SLOG_ERROR, hc->cfg.verbosity, 0,
            "%s: ERROR in SDHI%d, SD_INFO2 0x%08X , Need retune", __func__, sdmmc->chan_idx, info2);

        /*
         * Follow section 70.7.2 Sampling Clock Position Correction after Tuning
         * of document ver.0027, we need re-tuning for these errors
         */
        return -(EIO); /* Need re-tune */
    }

    if ((sdmmc_read(sdmmc->vbase, MMC_SCC_RVSCNTL) & MMC_SCC_RVSCNTL_RVSEN) != 0) {
        /* Automatic correction */
        return (rcar_sdmmc_scc_auto_correction(hc));
    } else {
        /* Manual correction */
        return (rcar_sdmmc_scc_manual_correction(hc));
    }
}

static int rcar_sdmmc_cd(sdio_hc_t *hc)
{
    const rcar_sdmmc_t *sdmmc;
    uint32_t            cstate, pstate;

    sdmmc  = hc->cs_hdl;
    cstate = CD_RMV;
    pstate = sdmmc_read(sdmmc->vbase, MMC_SD_INFO1);

    hc->caps |= HC_CAP_CD_INTR;

    if ((pstate & SDH_INFO1_CD) != 0) {
        cstate  |= CD_INS;
        if ((pstate & SDH_INFO1_WP) == 0) {
            cstate |= CD_WP;
        }
    }

    return (cstate);
}

static int rcar_sdmmc_dma_init(sdio_hc_t *hc)
{
    sdio_hc_cfg_t *cfg;

    cfg = &hc->cfg;
    if (cfg == NULL) {
        return (ENOMEM);
    }
    cfg->sg_max = 1;

    return (EOK);
}

int rcar_sdmmc_dinit(sdio_hc_t *hc)
{
    rcar_sdmmc_t *sdmmc;

    if ((hc == NULL) || (hc->cs_hdl == NULL)) {
        return (EOK);
    }

    sdmmc = hc->cs_hdl;

    if (sdmmc->vbase) {
        if (hc->hc_iid != -1) {
            InterruptDetach(hc->hc_iid);
        }

        munmap_device_io(sdmmc->vbase, sdmmc->basesize);
    }

#ifdef RCAR_CPG
    cpg_hwfuncs.deinit();
#endif

#if defined(VARIANT_ipmmu)
    rcar_ipmmu_cleanup_obj(sdmmc);
#endif /* VARIANT_ipmmu */

    free(sdmmc);
    hc->cs_hdl = NULL;

    return (EOK);
}

static void rcar_sdmmc_resume(sdio_hc_t * const hc)
{
	sdio_slogf(_SLOGC_SDIODI, _SLOG_INFO, hc->cfg.verbosity, 1, "%s ", __func__);
}

static void rcar_sdmmc_suspend(sdio_hc_t * const hc)
{
	sdio_slogf(_SLOGC_SDIODI, _SLOG_INFO, hc->cfg.verbosity, 1, "%s ", __func__);
}

static int rcar_sdmmc_pm(sdio_hc_t * const hc, const uint32_t action)
{
	switch( action ) {
		case PM_IDLE:
			break;
		case PM_ACTIVE:
			if ( hc->pm_state == PM_SUSPEND ) {
				rcar_sdmmc_resume(hc);
			} else if ( hc->pm_state == PM_SLEEP ) {
				// power on device
			} else {
				// Do nothing
			}
			break;
		case PM_SLEEP:
				// power off device
			break;
		case PM_SUSPEND:
			rcar_sdmmc_suspend(hc);
			break;
		default:
			break;
	}

	return (EOK);
}

static int rcar_sdmmc_reset(sdio_hc_t * const hc)
{
    rcar_sdmmc_t    *sdmmc = hc->cs_hdl;
    int timeout = 1000;

    /* Apply a soft reset */
    sdmmc_write(sdmmc->vbase, MMC_SOFT_RST, sdmmc_read(sdmmc->vbase, MMC_SOFT_RST) & ~SOFT_RST_OFF);
    sdmmc_write(sdmmc->vbase, MMC_SOFT_RST, sdmmc_read(sdmmc->vbase, MMC_SOFT_RST) | SOFT_RST_OFF);

    /* Check reset release state */
    while(((sdmmc_read(sdmmc->vbase, MMC_SOFT_RST) & SOFT_RST_OFF) == 0) && (timeout > 0)) {
        nanospin_ns(1000);
        timeout--;
    }

    sdmmc_write(sdmmc->vbase, MMC_SD_INFO1_MASK, SDH_INFO1_MASK_ALL);
    sdmmc_write(sdmmc->vbase, MMC_SD_INFO2_MASK, SDH_INFO2_MASK_ALL);
    sdmmc_write(sdmmc->vbase, MMC_HOST_MODE, 0x00000000);    // SD_BUF access width = 64-bit
    sdmmc_write(sdmmc->vbase, MMC_SD_OPTION, 0x0000C0EE);    // Bus width = 1bit, timeout=MAX
    sdmmc_write(sdmmc->vbase, MMC_SD_CLK_CTRL, 0x00000080);  // Automatic Control=Disable, Clock Output=Disable

    /* Mask all DMA interrupts */
    sdmmc_write(sdmmc->vbase, MMC_DM_CM_INFO1_MASK, DM_INFO1_ALL_MSK);
    sdmmc_write(sdmmc->vbase, MMC_DM_CM_INFO2_MASK, DM_INFO2_ALL_MSK);

    /* Reset SCC */
    sdmmc_write(sdmmc->vbase, MMC_SCC_CKSEL, ~MMC_SCC_CKSEL_DTSEL & sdmmc_read(sdmmc->vbase, MMC_SCC_CKSEL));
    sdmmc_write(sdmmc->vbase, MMC_SCC_RVSCNTL, ~MMC_SCC_RVSCNTL_RVSEN & sdmmc_read(sdmmc->vbase, MMC_SCC_RVSCNTL));
    sdmmc_write(sdmmc->vbase, MMC_SCC_DTCNTL, ~MMC_SCC_DTCNTL_TAPEN & sdmmc_read(sdmmc->vbase, MMC_SCC_DTCNTL));

    /* Reset HS400 mode */
    sdmmc_write(sdmmc->vbase, MMC_SDIF_MODE, sdmmc_read(sdmmc->vbase, MMC_SDIF_MODE) & ~SDIF_MODE_HS400);
    sdmmc_write(sdmmc->vbase, MMC_SCC_TMPPORT2,  sdmmc_read(sdmmc->vbase, MMC_SCC_TMPPORT2) &
                ~(MMC_SCC_TMPPORT2_HS400OSEL | MMC_SCC_TMPPORT2_HS400EN));

    if (sdmmc->adjust_hs400_enable) {
        rcar_sdmmc_hs400_adjust_disable(hc);
    }

    sdmmc->tuning_status = RCAR_TUNING_NONE;

    sdmmc->busclk = 400 * 1000;      // 400KHz clock for ident

    /* configure clock */
    rcar_sdmmc_clk(hc, sdmmc->busclk);

    return (EOK);
}

static void rcar_sdmmc_parse_hs400_support(sdio_hc_t *hc)
{
    rcar_sdmmc_t  *sdmmc = hc->cs_hdl;

    sdmmc->tap                      = 0x00000300;
    sdmmc->tap_hs400                = 0x00000300;
    sdmmc->hs400_use_4tap           = 0;
    sdmmc->dma_tranend1             = 20;
    sdmmc->adjust_hs400_calib_table = NULL;
    sdmmc->sdckcr_val_for_hs400     = 1;
    sdmmc->adjust_hs400_enable      = 0;
    sdmmc->adjust_hs400_offset      = MMC_SCC_TMPPORT3_OFFSET_0;

    hc->caps |= (uint64_t)HC_CAP_HS400;

    /* R-Car Gen4 SCC doesn't use manual sampling clock position correction in HS400 */
    sdmmc->hs400_manual_correction = 0;

    /* R-Car Gen4 SCC doesn't use DAT signal correction error status in HS400*/
    sdmmc->hs400_ignore_dat_correction = 0;

}

int rcar_sdmmc_init(sdio_hc_t *hc)
{
    sdio_hc_entry_t const rcar_sdmmc_entry = {
    .nentries                 = SDIO_HC_ENTRY_NFUNCS,
    .dinit                    = rcar_sdmmc_dinit,
    .pm                       = rcar_sdmmc_pm,
    .cmd                      = rcar_sdmmc_cmd,
    .abort                    = rcar_sdmmc_abort,
    .event                    = rcar_sdmmc_event,
    .cd                       = rcar_sdmmc_cd,
    .pwr                      = rcar_sdmmc_pwr,
    .clk                      = rcar_sdmmc_clk,
    .bus_mode                 = rcar_sdmmc_bus_mode,
    .bus_width                = rcar_sdmmc_bus_width,
    .timing                   = rcar_sdmmc_timing,
    .signal_voltage           = rcar_sdmmc_signal_voltage,
    .drv_type                 = NULL,
    .driver_strength          = NULL,
    .tune                     = rcar_sdmmc_tune,
    .preset                   = NULL
    };

    const sdio_hc_cfg_t   * cfg;
    rcar_sdmmc_t    *sdmmc;
    struct sigevent event;

    hc->hc_iid  = -1;
    cfg         = &hc->cfg;

    memcpy(&hc->entry, &rcar_sdmmc_entry, sizeof(sdio_hc_entry_t));

    hc->cs_hdl = calloc(1, sizeof(rcar_sdmmc_t));
    sdmmc = hc->cs_hdl;
    if (sdmmc == NULL) {
        return (ENOMEM);
    }

#if defined(VARIANT_ipmmu)
    /* "sdhci" is specified in librcar-ipmmu implementation */
    sdmmc->ipmmu_handle = ipmmu_open("sdhci", 0);
    if (sdmmc->ipmmu_handle < 0) {
        int err = errno;
        sdio_slogf(_SLOGC_SDIODI, _SLOG_ERROR, hc->cfg.verbosity, 0,
                "%s: Unable to ipmmu_open err=%s", __func__, strerror(err));
        return err;
    }
    TAILQ_INIT(&ipmmu_tailq_head);
    sdio_slogf(_SLOGC_SDIODI, _SLOG_INFO, hc->cfg.verbosity, 0,
            "%s: IPMMU enabled for address translation", __func__);
#endif /* defined(VARIANT_ipmmu) */

    if ((cfg->base_addrs > 0) && (cfg->irqs > 0)) {
        sdmmc->pbase = cfg->base_addr[0];
        sdmmc->basesize = (uint32_t)cfg->base_addr_size[0];
        sdmmc->irq   = cfg->irq[0];
        sdmmc->chan_idx = cfg->idx;
    } else {
        rcar_sdmmc_dinit(hc);
        return (ENODEV);
    }

    sdmmc->vbase = (uintptr_t)mmap_device_io(sdmmc->basesize, sdmmc->pbase);
    if (sdmmc->vbase == (uintptr_t)MAP_FAILED) {
        sdio_slogf(_SLOGC_SDIODI, _SLOG_ERROR, hc->cfg.verbosity, 0,
            "%s: Unable to mmap_device_io (SDHI base 0x%lx) %s", __func__, sdmmc->pbase, strerror(errno));
        rcar_sdmmc_dinit(hc);
        return (errno);
    }

    if (cfg->clk) {
        sdmmc->pclk = cfg->clk;
    } else {
        sdmmc->pclk = 200 * 1000 * 1000;
    }

#ifdef RCAR_CPG
    if (cpg_mgr_getfuncs(&cpg_hwfuncs, (int)sizeof(cpg_mgr_funcs_t))) {
        sdio_slogf(_SLOGC_SDIODI, _SLOG_ERROR, hc->cfg.verbosity, 0,
            "%s : Unable to get cpg hw funcs", __func__);
         return (ENODEV);
    }
#endif

    /*
     * In R-Car SDHI, internal SCC modules uses SDnH clock.
     * It is controlled by SDnCKCR.STPnHCK bit in CPG.
     * When SD clock is less than High Speed, SDnH clock is stopped.
     * And SDnH clock is supplied with 100MHz or more in Clock divider
     * table of CPG in R-Car Gen4.
     * It is the recommended setting of H/W.
     */
    sdmmc->scc_min_clk = 100 * 1000 * 1000;

    hc->clk_max = sdmmc->pclk;

    hc->caps |= HC_CAP_BSY | HC_CAP_BW4 | HC_CAP_BW8 | HC_CAP_CD_WP;
    hc->caps |= HC_CAP_ACMD12;
    hc->caps |= HC_CAP_DMA;
    hc->caps |= HC_CAP_HS | HC_CAP_HS200 | HC_CAP_SDR50 | HC_CAP_SDR104;

    rcar_sdmmc_parse_hs400_support(hc);

    hc->caps &= cfg->caps;      /* reconcile command line options */

    if (rcar_sdmmc_dma_init(hc) != EOK) {
        rcar_sdmmc_dinit(hc);
        return (ENODEV);
    }

    rcar_sdmmc_reset(hc);

    /* we don't want this interrupt at the driver startup */
    while (sdmmc_read(sdmmc->vbase, MMC_SD_INFO1) & SDH_INFO1_INST) {
        sdmmc_write(sdmmc->vbase, MMC_SD_INFO1, (uint32_t)(~(SDH_INFO1_INST | SDH_INFO1_RMVL)));
    }

    /* Only enable the card insertion and removal interrupts */
    sdmmc_write(sdmmc->vbase, MMC_SD_INFO1_MASK,  sdmmc_read(sdmmc->vbase, MMC_SD_INFO1_MASK) & ~(SDH_INFO1_INST | SDH_INFO1_RMVL));

    SIGEV_PULSE_INIT(&event, hc->hc_coid, (short)hc->priority, HC_EV_INTR, NULL);
    hc->hc_iid = InterruptAttachEvent(sdmmc->irq, &event, _NTO_INTR_FLAGS_TRK_MSK);
    if (hc->hc_iid == -1) {
        rcar_sdmmc_dinit(hc);
        return (errno);
    }

    return (EOK);
}
#endif
