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
#include "controller/ble_ll_sched.h"
#include "controller/ble_ll_tmr.h"
#include "ble_ll_priv.h"
#include "ble_ll_cs_priv.h"

extern struct ble_ll_cs_sm *g_ble_ll_cs_sm_current;
int ble_phy_tx_cs_tone(uint16_t duration_usecs);
int ble_phy_rx_cs_tone(uint16_t duration_usecs);
void ble_phy_cs_tone_mode_set(uint8_t mode);

int
ble_ll_cs_tone_tx_start(struct ble_ll_cs_sm *cssm)
{
    int rc;
    struct ble_ll_cs_step_transmission *step = cssm->step_transmission;
    uint32_t cputime;
    uint8_t rem_us;
    uint8_t ll_state;

    ll_state = ble_ll_state_get();
    BLE_LL_ASSERT(ll_state == BLE_LL_STATE_STANDBY || ll_state == BLE_LL_STATE_CS);

    if (cssm->step_mode == BLE_LL_CS_MODE0) {
        /* Frequency measurement */
        ble_phy_cs_tone_mode_set(2);
    } else { /* BLE_LL_CS_MODE2 || BLE_LL_CS_MODE3 */
        /* Phase measurement */
        ble_phy_cs_tone_mode_set(1);
    }

    ble_ll_tx_power_set(g_ble_ll_tx_power);

    cputime = ble_ll_tmr_u2t_r(cssm->anchor_usecs, &rem_us);

    /* At transition the radio is already scheduled to start at the right time */
    if (ll_state != BLE_LL_STATE_CS) {
        rc = ble_phy_tx_set_start_time(cputime, rem_us);
        if (rc) {
            ble_ll_cs_proc_sync_lost(cssm);
            return 1;
        }
    }

    ble_phy_set_txend_cb(NULL, NULL);

    rc = ble_phy_tx_cs_tone(step->duration_usecs);
    if (rc) {
        ble_ll_cs_proc_sync_lost(cssm);
        return 1;
    }

    ble_ll_state_set(BLE_LL_STATE_CS);

    return 0;
}

int
ble_ll_cs_tone_rx_start(struct ble_ll_cs_sm *cssm)
{
    int rc;
    struct ble_ll_cs_step_transmission *step = cssm->step_transmission;
    uint32_t cputime;
    uint8_t ll_state;
    uint8_t rem_us;

    ll_state = ble_ll_state_get();
    BLE_LL_ASSERT(ll_state == BLE_LL_STATE_STANDBY || ll_state == BLE_LL_STATE_CS);

    if (cssm->step_mode == BLE_LL_CS_MODE0) {
        /* Frequency measurement */
        ble_phy_cs_tone_mode_set(2);
    } else { /* BLE_LL_CS_MODE2 || BLE_LL_CS_MODE3 */
        /* Phase measurement */
        ble_phy_cs_tone_mode_set(1);
    }

    cputime = ble_ll_tmr_u2t_r(cssm->anchor_usecs, &rem_us);

    if (ll_state != BLE_LL_STATE_CS) {
        /* At transition the radio is already scheduled to start at the right time */
        return 0;
    }

    rc = ble_phy_rx_set_start_time(cputime, rem_us);
    if (rc) {
        ble_ll_cs_proc_sync_lost(cssm);
        return 1;
    }

    ble_phy_rx_cs_tone(step->duration_usecs);

    ble_ll_state_set(BLE_LL_STATE_CS);

    return 0;
}

void
ble_ll_cs_tone_tx_end_cb(struct ble_ll_cs_sm *cssm)
{
    struct ble_ll_cs_step_transmission *step = cssm->step_transmission;
    uint8_t i;

    if (cssm->step_mode == BLE_LL_CS_MODE0) {
        /* TODO: Read measured frequency offset.
         * For now set "Frequency offset is not available".
         */
        cssm->step_result.measured_freq_offset = 0xC000;
    } else if (cssm->step_mode == BLE_LL_CS_MODE2 ||
               cssm->step_mode == BLE_LL_CS_MODE3) {
        for (i = 0; i < cssm->n_ap; ++i) {
            cssm->step_result.tone_pct[i] = 0;
            cssm->step_result.tone_quality_ind[i] = 0;
        }
    }

    cssm->anchor_usecs += step->duration_usecs + step->end_tifs;
    ble_ll_cs_proc_schedule_next_tx_or_rx(cssm);
}

void
ble_ll_cs_tone_rx_end_cb(struct ble_ll_cs_sm *cssm)
{
    struct ble_ll_cs_step_transmission *step = cssm->step_transmission;

    cssm->anchor_usecs += step->duration_usecs + step->end_tifs;
    ble_ll_cs_proc_schedule_next_tx_or_rx(cssm);
}

#endif /* BLE_LL_CHANNEL_SOUNDING */
