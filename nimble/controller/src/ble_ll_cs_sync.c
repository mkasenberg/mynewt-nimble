/*
 * Licensed to the Apache Software Foundation (ASF) under one
 * or more contributor license agreements.  See the NOTICE file
 * distributed with this work for additional information
 * regarding copyright ownership.  The ASF licenses this file
 * to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance
 * with the License.  You may obtain a copy of the License at
 *
 *  http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing,
 * software distributed under the License is distributed on an
 * "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY
 * KIND, either express or implied.  See the License for the
 * specific language governing permissions and limitations
 * under the License.
 */

#include <syscfg/syscfg.h>
#if MYNEWT_VAL(BLE_LL_CHANNEL_SOUNDING)
#include <stdint.h>
#include "controller/ble_ll.h"
#include "controller/ble_ll_conn.h"
#include "controller/ble_ll_sched.h"
#include "controller/ble_ll_tmr.h"
#include "controller/ble_hw.h"
#include "controller/ble_ll_hci.h"
#include "mbedtls/aes.h"
#include "ble_ll_priv.h"
#include "ble_ll_cs_priv.h"
#include "bs_tracing.h"

extern struct ble_ll_cs_supp_cap g_ble_ll_cs_local_cap;
extern struct ble_ll_cs_sm g_ble_ll_cs_sm[MYNEWT_VAL(BLE_MAX_CONNECTIONS)];
extern struct ble_ll_cs_sm *g_ble_ll_cs_sm_current;
extern int8_t g_ble_ll_tx_power;

static uint8_t
ble_ll_cs_sync_calc_seq_quality(struct ble_ll_cs_sm *cssm, uint8_t *rxpdu,
                                struct ble_mbuf_hdr *hdr)
{
    /* TODO: Check if all bits match the expected sequence.
     * For now returns "all bits match".
     */
    return 0;
}

static uint8_t
ble_ll_cs_sync_calc_nadm(struct ble_ll_cs_sm *cssm, uint8_t *rxpdu,
                         struct ble_mbuf_hdr *hdr)
{
    uint8_t nadm;

    if (cssm->active_config->rtt_type == 0x00) {
        /* No sequence, so NADM unknown. */
        nadm = 0xFF;
    } else {
        /* TODO: Estimate how much a received GFSK modulated packet signal
         * differs from the expected packet signal. Use the normalized attack
         * detector metric (NADM). For now return "Attack extremely unlikely".
         */
        nadm = 0x00;
    }

    return nadm;
}

static uint8_t
ble_ll_cs_sync_make(struct ble_ll_cs_sm *cssm, uint8_t *buf)
{
    int i;
    int rc;
    uint32_t aa;
    struct ble_ll_cs_config *conf = cssm->active_config;
    uint8_t sequence_len = 0;
    uint8_t bits;

    if (conf->rtt_type != BLE_LL_CS_RTT_AA_ONLY &&
        cssm->step_mode != BLE_LL_CS_MODE0) {
        rc = ble_ll_cs_drbg_generate_sync_sequence(
            &cssm->drbg_ctx, cssm->steps_in_procedure_count, conf->rtt_type,
            buf, &sequence_len);

        if (rc) {
            return 0;
        }
    }

    /* Shift by 4 bits to make space for trailer bits */
    if (sequence_len > 0) {
        BLE_LL_ASSERT(sequence_len < BLE_PHY_MAX_PDU_LEN);
        buf[sequence_len] = 0;
        bits = 0;
        for (i = sequence_len - 1; i >= 0; --i) {
            bits = buf[i] >> 4;
            buf[i] = (buf[i] << 4) & 0xF0;
            buf[i + 1] = (buf[i + 1] & 0xF0) | bits;
        }
    }

    if (conf->role == BLE_LL_CS_ROLE_INITIATOR) {
        aa = cssm->initiator_aa;
    } else {
        aa = cssm->reflector_aa;
    }

    /* Set the trailer bits */
    if (aa & 0x80000000) {
        buf[0] |= 0x0A;
    } else {
        buf[0] |= 0x05;
    }
    ++sequence_len;

    return sequence_len;
}

/**
 * Called when a CS pkt has been transmitted.
 * Schedule next CS pkt reception.
 *
 * Context: interrupt
 */
void
ble_ll_cs_sync_tx_end_cb(void *arg)
{
    struct ble_ll_cs_sm *cssm = (struct ble_ll_cs_sm *)arg;
    uint32_t cputime;
    uint32_t rem_us;
    uint32_t rem_ns;
    uint32_t end_anchor_usecs;

    bs_trace_raw_time(0, "End of CS SYNC transmission\n");

    BLE_LL_ASSERT(cssm != NULL);

    ble_phy_get_txend_time(&cputime, &rem_us, &rem_ns);
    end_anchor_usecs = ble_ll_tmr_t2u(cputime) + rem_us;

    cssm->step_result.time_of_departure_us = end_anchor_usecs;
    cssm->step_result.time_of_departure_ns = rem_ns;

    bs_trace_raw_time(0, "End of CS SYNC TX: %u[us] = %u[ticks] + %u[us] = %u[us] + %u[us]\n",
                      end_anchor_usecs,
                      cputime, rem_us,
                      ble_ll_tmr_t2u(cputime), rem_us);

    cssm->anchor_usecs = end_anchor_usecs + cssm->step_transmission->end_tifs;
    ble_ll_cs_proc_schedule_next_tx_or_rx(cssm);
}

static uint8_t
ble_ll_cs_sync_tx_make(uint8_t *dptr, void *arg, uint8_t *hdr_byte)
{
    uint8_t pdu_len;
    struct ble_ll_cs_sm *cssm = arg;

    /* TODO: Unused fields in CS Sync packet */
    pdu_len = ble_ll_cs_sync_make(cssm, dptr);
    *hdr_byte = 0;

    return pdu_len;
}

int
ble_ll_cs_sync_tx_start(struct ble_ll_cs_sm *cssm)
{
    int rc;
    uint32_t cputime;
    struct ble_ll_cs_step_transmission *step = cssm->step_transmission;
    uint8_t rem_us;
    uint8_t ll_state;

    ll_state = ble_ll_state_get();
    BLE_LL_ASSERT(ll_state == BLE_LL_STATE_STANDBY || ll_state == BLE_LL_STATE_CS);

    bs_trace_raw_time(0, "Starting CS SYNC TX: %u[us] = %u[ticks] + %u[us] = %u[us] + %u[us]\n",
                      ble_ll_tmr_t2u(ble_ll_tmr_get()) + 0,
                      ble_ll_tmr_get(), 0,
                      ble_ll_tmr_t2u(ble_ll_tmr_get()), 0);

    ble_phy_cs_sync_mode_set(1);

    ble_ll_tx_power_set(g_ble_ll_tx_power);

    rc = ble_phy_cs_sync_configure(cssm->channel, cssm->tx_aa);
    if (rc) {
        ble_ll_cs_proc_sync_lost(cssm);
        return 1;
    }

    cputime = ble_ll_tmr_u2t_r(cssm->anchor_usecs, &rem_us);

    /* At transition the radio is already scheduled to start at the right time */
    if (ll_state != BLE_LL_STATE_CS) {
        ble_phy_transition_set(step->end_transition, step->end_tifs);

        rc = ble_phy_tx_set_start_time(cputime, rem_us);
        if (rc) {
            ble_ll_cs_proc_sync_lost(cssm);
            return 1;
        }
    }

    bs_trace_raw_time(0, "Radio TX start: %u[us] = %u[ticks] + %u[us] = %u[us] + %u[us]\n",
                      ble_ll_tmr_t2u(cputime) + rem_us,
                      cputime, rem_us,
                      ble_ll_tmr_t2u(cputime), rem_us);

    ble_phy_set_txend_cb(ble_ll_cs_sync_tx_end_cb, cssm);

    rc = ble_phy_tx_cs_sync(ble_ll_cs_sync_tx_make, cssm);
    if (rc) {
        ble_ll_cs_proc_sync_lost(cssm);
        return 1;
    }

    ble_ll_state_set(BLE_LL_STATE_CS);

    return 0;
}

int
ble_ll_cs_sync_rx_start(struct ble_ll_cs_sm *cssm)
{
    int rc;
    uint32_t cputime;
    struct ble_ll_cs_step_transmission *step = cssm->step_transmission;
    uint8_t ll_state;
    uint8_t rem_us;

    ll_state = ble_ll_state_get();
    BLE_LL_ASSERT(ll_state == BLE_LL_STATE_STANDBY || ll_state == BLE_LL_STATE_CS);

    bs_trace_raw_time(0, "Starting CS SYNC RX: %u[us] = %u[ticks] + %u[us] = %u[us] + %u[us]\n",
                      ble_ll_tmr_t2u(ble_ll_tmr_get()) + 0,
                      ble_ll_tmr_get(), 0,
                      ble_ll_tmr_t2u(ble_ll_tmr_get()), 0);

    ble_phy_cs_sync_mode_set(1);

    rc = ble_phy_cs_sync_configure(cssm->channel, cssm->rx_aa);
    if (rc) {
        ble_ll_cs_proc_sync_lost(cssm);
        return 1;
    }

    cputime = ble_ll_tmr_u2t_r(cssm->anchor_usecs, &rem_us);

    bs_trace_raw_time(0, "Radio RX start: %u[us] = %u[ticks] + %u[us] = %u[us] + %u[us]\n",
                     ble_ll_tmr_t2u(cputime) + rem_us,
                     cputime, rem_us,
                     ble_ll_tmr_t2u(cputime), rem_us);

    if (ll_state == BLE_LL_STATE_CS) {
        /* At transition the radio is already scheduled to start at the right time */
        return 0;
    }

    ble_phy_transition_set(step->end_transition, step->end_tifs);

    rc = ble_phy_rx_set_start_time(cputime, rem_us);
    if (rc) {
        ble_ll_cs_proc_sync_lost(cssm);
        return 1;
    }

    ble_phy_wfr_enable(BLE_PHY_WFR_ENABLE_RX, 0, step->wfr_usecs);
    ble_ll_state_set(BLE_LL_STATE_CS);
    return 0;
}

static struct ble_ll_cs_sm *
ble_ll_cs_sync_find_sm_match_aa(uint32_t aa)
{
    int i;
    struct ble_ll_cs_sm *cssm;

    for (i = 0; i < ARRAY_SIZE(g_ble_ll_cs_sm); ++i) {
        cssm = &g_ble_ll_cs_sm[i];

        if (cssm->rx_aa == aa) {
            return cssm;
        }
    }

    return NULL;
}

/**
 * Called when a receive PDU has started.
 * Check if the frame is the next expected CS_SYNC packet.
 *
 * Context: interrupt
 *
 * @return int
 *   < 0: A frame we dont want to receive.
 *   = 0: Continue to receive frame. Dont go from rx to tx
 */
int
ble_ll_cs_sync_rx_isr_start(struct ble_mbuf_hdr *rxhdr, uint32_t aa)
{
    struct ble_ll_cs_sm *cssm;

    cssm = ble_ll_cs_sync_find_sm_match_aa(aa);

    if (cssm == NULL) {
        /* This is not the expected packet. Skip the frame. */
        return -1;
    }

    return 0;
}

/**
 * Called when received a complete CS_SYNC packet.
 *
 * Context: Interrupt
 *
 * @param rxpdu
 * @param rxhdr
 *
 * @return int
 *       < 0: Disable the phy after reception.
 *      == 0: Success. Do not disable the PHY.
 *       > 0: Do not disable PHY as that has already been done.
 */
int
ble_ll_cs_sync_rx_isr_end(uint8_t *rxbuf, struct ble_mbuf_hdr *rxhdr)
{
    struct ble_mbuf_hdr *ble_hdr;
    struct os_mbuf *rxpdu;
    struct ble_ll_cs_sm *cssm = g_ble_ll_cs_sm_current;
    uint32_t cputime;
    uint32_t rem_us;
    uint32_t rem_ns;
    uint32_t end_anchor_usecs;

    /* Packet type was verified in isr_start */

    BLE_LL_ASSERT(cssm != NULL);

    ble_phy_get_rxend_time(&cputime, &rem_us, &rem_ns);
    end_anchor_usecs = ble_ll_tmr_t2u(cputime) + rem_us;

    bs_trace_raw_time(0, "End of CS SYNC reception, t=%d+%d=%d\n",
                      ble_ll_tmr_t2u(cputime), rem_us,
                      end_anchor_usecs);

    cssm->step_result.time_of_arrival_us = end_anchor_usecs;
    cssm->step_result.time_of_arrival_ns = rem_ns;
    cssm->step_result.packet_rssi = rxhdr->rxinfo.rssi;
    cssm->step_result.packet_quality =
        ble_ll_cs_sync_calc_seq_quality(cssm, rxbuf, rxhdr);
    cssm->step_result.packet_nadm =
        ble_ll_cs_sync_calc_nadm(cssm, rxbuf, rxhdr);

    if (g_ble_ll_cs_local_cap.sounding_pct_estimate &&
        cssm->active_config->rtt_type != BLE_LL_CS_RTT_AA_ONLY) {
        /* TODO: Read PCT estimates from sounding sequence.
         * For now set "Phase Correction Term is not available".
         */
        cssm->step_result.packet_pct1 = 0xFFFFFFFF;
        cssm->step_result.packet_pct2 = 0xFFFFFFFF;
    }

    cssm->anchor_usecs = end_anchor_usecs + cssm->step_transmission->end_tifs;
    ble_ll_cs_proc_schedule_next_tx_or_rx(cssm);

    ble_hdr = BLE_MBUF_HDR_PTR(rxpdu);
    ble_hdr->rxinfo.user_data = cssm;

    rxpdu = ble_ll_rxpdu_alloc(rxbuf[1] + BLE_LL_PDU_HDR_LEN);
    if (rxpdu) {
        ble_phy_rxpdu_copy(rxbuf, rxpdu);

        /* Send the packet to Link Layer context */
        ble_ll_rx_pdu_in(rxpdu);
    }

    return 1;
}

/**
 * Process a received PDU.
 *
 * Context: Link Layer task.
 *
 * @param pdu_type
 * @param rxbuf
 */
void
ble_ll_cs_sync_rx_pkt_in(struct os_mbuf *rxpdu, struct ble_mbuf_hdr *rxhdr)
{
    struct ble_ll_cs_sm *cssm = rxhdr->rxinfo.user_data;

    bs_trace_raw_time(0, "Received CS SYNC\n");
}

#endif /* BLE_LL_CHANNEL_SOUNDING */
