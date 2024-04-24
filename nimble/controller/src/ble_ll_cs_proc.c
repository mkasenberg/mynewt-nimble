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
#include "controller/ble_ll_utils.h"
#include "controller/ble_ll_conn.h"
#include "controller/ble_ll_sched.h"
#include "controller/ble_ll_tmr.h"
#include "ble_ll_cs_priv.h"

extern struct ble_ll_cs_supp_cap g_ble_ll_cs_local_cap;
extern uint8_t g_ble_ll_cs_chan_count;
extern uint8_t g_ble_ll_cs_chan_indices[72];
struct ble_ll_cs_sm *g_ble_ll_cs_sm_current;

#define SUBEVENT_STATE_MODE0_STEP      (0)
#define SUBEVENT_STATE_REPETITION_STEP (1)
#define SUBEVENT_STATE_SUBMODE_STEP    (2)
#define SUBEVENT_STATE_MAINMODE_STEP   (3)

#define CS_SCHEDULE_NEW_STEP      (0)
#define CS_SCHEDULE_NEW_SUBEVENT  (1)
#define CS_SCHEDULE_NEW_EVENT     (2)
#define CS_SCHEDULE_NEW_PROCEDURE (3)

#define BLE_LL_CONN_ITVL_USECS    (1250)

/* The ramp-down window in µs */
#define T_RD (5)
/* The guard time duration in µs */
#define T_GD (10)
/* The requency measurement period in µs */
#define T_FM (80)

static struct ble_ll_cs_aci aci_table[] = {
    {1, 1, 1}, {2, 2, 1}, {3, 3, 1}, {4, 4, 1},
    {2, 1, 2}, {3, 1, 3}, {4, 1, 4}, {4, 2, 2}
};
static const uint8_t rtt_seq_len[] = {0, 4, 12, 4, 8, 12, 16};
static uint32_t tifs_table[6][6];

/* A pattern containing the states and transitions of the current step */
static struct ble_ll_cs_step_transmission transmission_pattern[12];

void ble_ll_cs_tone_tx_end_cb(struct ble_ll_cs_sm *cssm);
void ble_ll_cs_tone_rx_end_cb(struct ble_ll_cs_sm *cssm);

static int
ble_ll_cs_generate_channel(struct ble_ll_cs_sm *cssm)
{
    int rc;
    uint8_t transaction_id;
    uint8_t *channel_array;
    uint8_t *next_channel_id;

    if (cssm->step_mode == BLE_LL_CS_MODE0) {
        transaction_id = BLE_LL_CS_DRBG_HOP_CHAN_MODE0;
        channel_array = cssm->mode0_channels;
        next_channel_id = &cssm->mode0_next_chan_id;
    } else {
        transaction_id = BLE_LL_CS_DRBG_HOP_CHAN_NON_MODE0;
        channel_array = cssm->non_mode0_channels;
        next_channel_id = &cssm->non_mode0_next_chan_id;
    }

    if (g_ble_ll_cs_chan_count <= *next_channel_id) {
        rc = ble_ll_cs_drbg_shuffle_cr1(&cssm->drbg_ctx, cssm->steps_in_procedure_count,
                                        transaction_id, g_ble_ll_cs_chan_indices,
                                        channel_array, g_ble_ll_cs_chan_count);
        if (rc) {
            return rc;
        }

        *next_channel_id = 0;
    }

    cssm->channel = channel_array[(*next_channel_id)++];

    return 0;
}

/**
 * Called when scheduled event needs to be halted. This normally should not be called
 * and is only called when a scheduled item executes but scanning for sync/chain
 * is stil ongoing
 * Context: Interrupt
 */
void
ble_ll_cs_proc_halt(void)
{
}

/**
 * Called when a scheduled event has been removed from the scheduler
 * without being run.
 */
void
ble_ll_cs_proc_rm_from_sched(void *cb_args)
{
}

static int
ble_lL_cs_validate_step_duration(struct ble_ll_cs_sm *cssm)
{
    const struct ble_ll_cs_config *conf = cssm->active_config;
    const struct ble_ll_cs_proc_params *params = &conf->proc_params;
    uint32_t max_procedure_len = conf->proc_params.max_procedure_len;
    uint32_t max_procedure_len_usecs = max_procedure_len * BLE_LL_CS_PROCEDURE_LEN_UNIT_US;
    uint32_t max_subevent_len_usecs = conf->proc_params.subevent_len;
    uint32_t step_duration_usecs = cssm->mode_duration_usecs[cssm->step_mode];
    uint32_t total_subevent_usecs = cssm->step_anchor_usecs - cssm->subevent_anchor_usecs;
    uint32_t total_procedure_usecs = cssm->step_anchor_usecs - cssm->procedure_anchor_usecs;

    if (total_procedure_usecs + step_duration_usecs > max_procedure_len_usecs ||
        cssm->steps_in_procedure_count + 1 >= BLE_LL_CS_STEPS_PER_PROCEDURE_MAX) {
        /* The CS procedure is complete */

        /* A CS procedure is considered complete and closed when at least one of the following
         * conditions occurs:
         * • The execution of the next CS step in its entirety would cause the extent of the CS
         *   procedure to exceed T_MAX_PROCEDURE_LEN.
         * • The combined number of mode-0 steps and non-mode‑0 steps executed is equal to
         *   N_STEPS_MAX as described in [Vol 6] Part B, Section 4.5.18.1.
         * • The number of CS subevents executed is equal to
         *   N_MAX_SUBEVENTS_PER_PROCEDURE.
         * TODO:
         * • If Channel Selection Algorithm #3b is used for non-mode-0 steps, and the channel
         *   map generated from CSFilteredChM has been used for CSNumRepetitions cycles
         *   for non-mode‑0 steps including the use for both Main_Mode and Sub_Mode steps.
         * • If Channel Selection Algorithm #3c is used for non-mode-0 steps, and
         *   CSNumRepetitions invocations of Channel Selection Algorithm #3c have been
         *   completed for non-mode‑0 steps including the use for both Main_Mode and
         *   Sub_Mode steps.
         */

        return CS_SCHEDULE_NEW_PROCEDURE;
    }

    if (total_subevent_usecs + step_duration_usecs > max_subevent_len_usecs ||
        cssm->steps_in_subevent_count + 1 >= BLE_LL_CS_STEPS_PER_SUBEVENT_MAX) {

        if (cssm->subevents_in_procedure_count + 1 >= BLE_LL_CS_SUBEVENTS_PER_PROCEDURE_MAX) {
            return CS_SCHEDULE_NEW_PROCEDURE;
        }

        if (cssm->subevents_in_event_count + 1 >= params->subevents_per_event) {
            total_procedure_usecs = cssm->event_anchor_usecs + cssm->event_interval_usecs +
                                    cssm->mode_duration_usecs[BLE_LL_CS_MODE0] -
                                    cssm->procedure_anchor_usecs;

            if (total_procedure_usecs > max_procedure_len_usecs) {
                return CS_SCHEDULE_NEW_PROCEDURE;
            }

            return CS_SCHEDULE_NEW_EVENT;
        }

        return CS_SCHEDULE_NEW_SUBEVENT;
    }

    return CS_SCHEDULE_NEW_STEP;
}

static int
ble_ll_cs_setup_next_subevent(struct ble_ll_cs_sm *cssm)
{
    ++cssm->subevents_in_procedure_count;
    ++cssm->subevents_in_event_count;
    cssm->steps_in_subevent_count = 0;

    cssm->subevent_anchor_usecs += cssm->event_interval_usecs;
    cssm->step_anchor_usecs = cssm->subevent_anchor_usecs;

    return 0;
}

static int
ble_ll_cs_setup_next_event(struct ble_ll_cs_sm *cssm)
{
    ++cssm->events_in_procedure_count;
    ++cssm->subevents_in_procedure_count;
    cssm->subevents_in_event_count = 0;
    cssm->steps_in_subevent_count = 0;

    cssm->event_anchor_usecs += cssm->event_interval_usecs;
    cssm->subevent_anchor_usecs = cssm->event_anchor_usecs;
    cssm->step_anchor_usecs = cssm->event_anchor_usecs;

    return 0;
}

static int
ble_ll_cs_setup_next_procedure(struct ble_ll_cs_sm *cssm)
{
    ++cssm->procedure_count;
    cssm->events_in_procedure_count = 0;
    cssm->subevents_in_procedure_count = 0;
    cssm->subevents_in_event_count = 0;
    cssm->steps_in_procedure_count = 0;
    cssm->steps_in_subevent_count = 0;

    cssm->procedure_anchor_usecs += cssm->procedure_anchor_usecs;
    cssm->step_anchor_usecs = cssm->procedure_anchor_usecs;
    cssm->subevent_anchor_usecs = cssm->step_anchor_usecs;

    return 0;
}

static int
ble_ll_cs_proc_subevent_next_state(struct ble_ll_cs_sm *cssm)
{
    int rc;
    const struct ble_ll_cs_config *conf = cssm->active_config;

    if (cssm->steps_in_subevent_count == 0) {
        ble_ll_cs_drbg_clear_cache(&cssm->drbg_ctx);
        cssm->mode0_step_count = conf->mode_0_steps;

        if (cssm->subevents_in_procedure_count == 0) {
            cssm->main_step_count = 0xFF;
        } else {
            cssm->repetition_count = conf->main_mode_repetition;
        }
    }

    if (cssm->mode0_step_count > 0) {
        cssm->subevent_state = SUBEVENT_STATE_MODE0_STEP;
        cssm->step_mode = BLE_LL_CS_MODE0;
        --cssm->mode0_step_count;

    } else if (cssm->repetition_count > 0) {
        cssm->subevent_state = SUBEVENT_STATE_REPETITION_STEP;
        cssm->step_mode = conf->main_mode;
        --cssm->repetition_count;

    } else if (cssm->main_step_count == 0) {
        cssm->subevent_state = SUBEVENT_STATE_SUBMODE_STEP;
        cssm->step_mode = conf->sub_mode;
        cssm->main_step_count = 0xFF;

    } else {
        cssm->subevent_state = SUBEVENT_STATE_MAINMODE_STEP;
        cssm->step_mode = conf->main_mode;
    }

    rc = ble_lL_cs_validate_step_duration(cssm);
    if (rc) {
        /* Step does not fit in current subevent. */
        switch (rc) {
        case CS_SCHEDULE_NEW_SUBEVENT:
            ble_ll_cs_setup_next_subevent(cssm);
            break;
        case CS_SCHEDULE_NEW_EVENT:
            ble_ll_cs_setup_next_event(cssm);
            break;
        case CS_SCHEDULE_NEW_PROCEDURE:
            if (conf->proc_params.max_procedure_count <= cssm->procedure_count + 1) {
                /* All CS procedures have been completed */
                return 1;
            }

            ble_ll_cs_setup_next_procedure(cssm);
            break;
        default:
            BLE_LL_ASSERT(0);
        }

        cssm->step_mode = BLE_LL_CS_MODE0;

        if (ble_lL_cs_validate_step_duration(cssm) != CS_SCHEDULE_NEW_STEP) {
            return 1;
        }

        return 0;
    }

    if (cssm->subevent_state == SUBEVENT_STATE_MAINMODE_STEP &&
        conf->sub_mode != 0xFF) {
        if (cssm->main_step_count == 0xFF) {
            /* Rand the number of Main_Mode steps to execute
             * before a Sub_Mode insertion.
             */
            rc = ble_ll_cs_drbg_rand_main_mode_steps(
                &cssm->drbg_ctx, cssm->steps_in_procedure_count,
                conf->main_mode_min_steps, conf->main_mode_max_steps,
                &cssm->main_step_count);

            if (rc) {
                return rc;
            }
        }

        --cssm->main_step_count;
    }

    return 0;
}

static uint8_t
ble_ll_cs_proc_set_t_sw(struct ble_ll_cs_sm *cssm)
{
    uint8_t t_sw;
    uint8_t t_sw_i;
    uint8_t t_sw_r;
    uint8_t aci = cssm->active_config->proc_params.aci;

    if (cssm->active_config->role == BLE_LL_CS_ROLE_INITIATOR) {
        t_sw_i = g_ble_ll_cs_local_cap.t_sw;
        t_sw_r = cssm->remote_cap.t_sw;
    } else { /* BLE_LL_CS_ROLE_REFLECTOR */
        t_sw_i = cssm->remote_cap.t_sw;
        t_sw_r = g_ble_ll_cs_local_cap.t_sw;
    }

    if (aci == 0) {
        t_sw = 0;
    } else if (IN_RANGE(aci, 1, 3)) {
        t_sw = t_sw_i;
    } else if (IN_RANGE(aci, 4, 6)) {
        t_sw = t_sw_r;
    } else { /* ACI == 7 */
        if (g_ble_ll_cs_local_cap.t_sw > cssm->remote_cap.t_sw) {
            t_sw = g_ble_ll_cs_local_cap.t_sw;
        } else {
            t_sw = cssm->remote_cap.t_sw;
        }
    }

    cssm->t_sw = t_sw;

    return t_sw;
}

static void
ble_ll_cs_proc_tifs_init(uint32_t t_ip1, uint32_t t_ip2, uint32_t t_sw, uint32_t t_fcs)
{
    memset(tifs_table, 0, sizeof(tifs_table));

    tifs_table[STEP_STATE_CS_SYNC_I][STEP_STATE_CS_SYNC_R] = T_RD + t_ip1;
    tifs_table[STEP_STATE_CS_SYNC_I][STEP_STATE_CS_TONE_I] = T_GD;

    tifs_table[STEP_STATE_CS_SYNC_R][STEP_STATE_CS_TONE_R] = T_GD;
    tifs_table[STEP_STATE_CS_SYNC_R][STEP_STATE_COMPLETE] = T_RD + t_fcs;

    tifs_table[STEP_STATE_CS_TONE_I][STEP_STATE_CS_TONE_I] = t_sw;
    tifs_table[STEP_STATE_CS_TONE_I][STEP_STATE_CS_TONE_R] = T_RD + t_ip2;

    tifs_table[STEP_STATE_CS_TONE_R][STEP_STATE_CS_SYNC_R] = T_GD;
    tifs_table[STEP_STATE_CS_TONE_R][STEP_STATE_CS_TONE_R] = t_sw;
    tifs_table[STEP_STATE_CS_TONE_R][STEP_STATE_COMPLETE] = T_RD + t_fcs;
}

static uint32_t
ble_ll_cs_proc_tifs_get(uint8_t old_state, uint8_t new_state)
{
    BLE_LL_ASSERT(old_state < 6 && new_state < 6);
    return tifs_table[old_state][new_state];
}

static int
ble_ll_cs_proc_calculate_timing(struct ble_ll_cs_sm *cssm)
{
    struct ble_ll_cs_config *conf = cssm->active_config;
    const struct ble_ll_cs_proc_params *params = &conf->proc_params;
    uint8_t t_fcs = conf->t_fcs;
    uint8_t t_ip1 = conf->t_ip1;
    uint8_t t_ip2 = conf->t_ip2;
    uint8_t t_pm = conf->t_pm;
    uint8_t t_sw;
    uint8_t n_ap = cssm->n_ap;
    uint8_t t_sy;
    uint8_t t_sy_seq;
    uint8_t sequence_len;

    t_sw = ble_ll_cs_proc_set_t_sw(cssm);

    /* CS packets with no Sounding Sequence or Random Sequence fields take 44 µs
     * to transmit when sent using the LE 1M PHY and 26 µs to transmit when using
     * the LE 2M and the LE 2M 2BT PHYs. CS packets that include a Sounding Sequence
     * or Random Sequence field take proportionally longer to transmit based on
     * the length of the field and the PHY selection.
     */

    sequence_len = rtt_seq_len[conf->rtt_type];

    switch (conf->cs_sync_phy) {
    case BLE_LL_CS_SYNC_PHY_1M:
        t_sy = BLE_LL_CS_SYNC_TIME_1M;
        t_sy_seq = sequence_len;
        break;
    case BLE_LL_CS_SYNC_PHY_2M:
        t_sy = BLE_LL_CS_SYNC_TIME_2M;
        t_sy_seq = sequence_len / 2;
        break;
    default:
        BLE_LL_ASSERT(0);
    }

    cssm->mode_duration_usecs[BLE_LL_CS_MODE0] =
            t_ip1 + T_GD + T_FM + 2 * (t_sy + T_RD) + t_fcs;

    cssm->mode_duration_usecs[BLE_LL_CS_MODE1] =
            t_ip1 + 2 * (t_sy + t_sy_seq + T_RD) + t_fcs;

    cssm->mode_duration_usecs[BLE_LL_CS_MODE2] =
            t_ip2 + 2 * ((t_sw + t_pm) * (n_ap + 1) + T_RD) + t_fcs;

    cssm->mode_duration_usecs[BLE_LL_CS_MODE3] =
            t_ip2 + 2 * ((t_sy + t_sy_seq + T_GD + T_RD) +
            (t_sw + t_pm) * (n_ap + 1)) + t_fcs;

    cssm->t_sy = t_sy;
    cssm->t_sy_seq = t_sy_seq;

    cssm->subevent_interval_usecs = params->subevent_interval * BLE_LL_CS_SUBEVENTS_INTERVAL_UNIT_US;

    cssm->event_interval_usecs = params->event_interval * cssm->connsm->conn_itvl *
                                 BLE_LL_CONN_ITVL_USECS;

    cssm->procedure_interval_usecs = params->procedure_interval * cssm->connsm->conn_itvl *
                                     BLE_LL_CONN_ITVL_USECS;
    return 0;
}

static uint32_t
ble_ll_cs_proc_step_state_duration_get(uint8_t state, uint8_t mode,
                                       uint8_t t_sy, uint8_t t_sy_seq)
{
    uint32_t duration = 0;

    switch (state) {
    case STEP_STATE_CS_SYNC_I:
    case STEP_STATE_CS_SYNC_R:
        duration = t_sy;
        if (mode != BLE_LL_CS_MODE0) {
            duration += t_sy_seq;
        }
        break;
    case STEP_STATE_CS_TONE_I:
    case STEP_STATE_CS_TONE_R:
        duration = T_FM;
        break;
    case STEP_STATE_INIT:
    case STEP_STATE_COMPLETE:
        duration = 0;
        break;
    default:
        BLE_LL_ASSERT(0);
    }

    return duration;
}

static uint8_t
ble_ll_cs_proc_mode0_next_state(uint8_t state)
{
    switch (state) {
    case STEP_STATE_INIT:
        state = STEP_STATE_CS_SYNC_I;
        break;
    case STEP_STATE_CS_SYNC_I:
        state = STEP_STATE_CS_SYNC_R;
        break;
    case STEP_STATE_CS_SYNC_R:
        state = STEP_STATE_CS_TONE_R;
        break;
    case STEP_STATE_CS_TONE_R:
        state = STEP_STATE_COMPLETE;
        break;
    default:
        BLE_LL_ASSERT(0);
    }

    return state;
}

static uint8_t
ble_ll_cs_proc_mode1_next_state(uint8_t state)
{
    switch (state) {
    case STEP_STATE_INIT:
        state = STEP_STATE_CS_SYNC_I;
        break;
    case STEP_STATE_CS_SYNC_I:
        state = STEP_STATE_CS_SYNC_R;
        break;
    case STEP_STATE_CS_SYNC_R:
        state = STEP_STATE_COMPLETE;
        break;
    default:
        BLE_LL_ASSERT(0);
    }

    return state;
}

static uint8_t
ble_ll_cs_proc_mode2_next_state(uint8_t state, uint8_t slot_count)
{
    switch (state) {
    case STEP_STATE_INIT:
        state = STEP_STATE_CS_TONE_I;
        break;
    case STEP_STATE_CS_TONE_I:
        if (slot_count == 0) {
            state = STEP_STATE_CS_TONE_R;
        }
        break;
    case STEP_STATE_CS_TONE_R:
        if (slot_count == 0) {
            state = STEP_STATE_COMPLETE;
        }
        break;
    default:
        BLE_LL_ASSERT(0);
    }

    return state;
}

static uint8_t
ble_ll_cs_proc_mode3_next_state(uint8_t state, uint8_t slot_count)
{
    switch (state) {
    case STEP_STATE_INIT:
        state = STEP_STATE_CS_SYNC_I;
        break;
    case STEP_STATE_CS_SYNC_I:
        state = STEP_STATE_CS_TONE_I;
        break;
    case STEP_STATE_CS_TONE_I:
        if (slot_count == 0) {
            state = STEP_STATE_CS_TONE_R;
        }
        break;
    case STEP_STATE_CS_TONE_R:
        if (slot_count == 0) {
            state = STEP_STATE_CS_SYNC_R;
        }
        break;
    case STEP_STATE_CS_SYNC_R:
        state = STEP_STATE_COMPLETE;
        break;
    default:
        BLE_LL_ASSERT(0);
    }

    return state;
}

static uint8_t
ble_ll_cs_proc_next_step_state_get(uint8_t mode, uint8_t state, uint8_t slot_count)
{
    switch (mode) {
    case BLE_LL_CS_MODE0:
        state = ble_ll_cs_proc_mode0_next_state(state);
        break;
    case BLE_LL_CS_MODE1:
        state = ble_ll_cs_proc_mode1_next_state(state);
        break;
    case BLE_LL_CS_MODE2:
        state = ble_ll_cs_proc_mode2_next_state(state, slot_count);
        break;
    case BLE_LL_CS_MODE3:
        state = ble_ll_cs_proc_mode3_next_state(state, slot_count);
        break;
    default:
        BLE_LL_ASSERT(0);
    }

    return state;
}

static int
ble_ll_cs_proc_skip_txrx(struct ble_ll_cs_sm *cssm)
{
    struct ble_ll_cs_step_transmission *step = cssm->step_transmission;

    cssm->anchor_usecs += step->duration_usecs + step->end_tifs;
    ble_ll_cs_proc_schedule_next_tx_or_rx(cssm);

    return 0;
}

static ble_ll_cs_sched_cb_func
ble_ll_cs_proc_sched_cb_get(uint8_t role, uint8_t step_state)
{
    ble_ll_cs_sched_cb_func cb;
    bool is_initiator = (role == BLE_LL_CS_ROLE_INITIATOR);

    switch (step_state) {
    case STEP_STATE_CS_SYNC_I:
        cb = is_initiator ? ble_ll_cs_sync_tx_start : ble_ll_cs_sync_rx_start;
        break;
    case STEP_STATE_CS_SYNC_R:
        cb = is_initiator ? ble_ll_cs_sync_rx_start : ble_ll_cs_sync_tx_start;
        break;
    case STEP_STATE_CS_TONE_I:
        cb = is_initiator ? ble_ll_cs_tone_tx_start : ble_ll_cs_tone_rx_start;
        break;
    case STEP_STATE_CS_TONE_R:
        cb = is_initiator ? ble_ll_cs_tone_rx_start : ble_ll_cs_tone_tx_start;
        break;
    default:
        BLE_LL_ASSERT(0);
    }

    return cb;
}

static uint8_t
ble_ll_cs_proc_transition_get(ble_ll_cs_sched_cb_func cb)
{
    uint8_t transition;

    if (cb == ble_ll_cs_sync_tx_start || cb == ble_ll_cs_tone_tx_start) {
        transition = BLE_PHY_TRANSITION_TO_TX;
    } else if (cb == ble_ll_cs_sync_rx_start || cb == ble_ll_cs_tone_rx_start) {
        transition = BLE_PHY_TRANSITION_TO_RX;
    } else {
        transition = BLE_PHY_TRANSITION_NONE;
    }

    return transition;
}

static int
ble_ll_cs_proc_trans_pattern_generate(struct ble_ll_cs_sm *cssm)
{
    struct ble_ll_cs_step_transmission *step;
    struct ble_ll_cs_step_transmission *prev_step;
    ble_ll_cs_sched_cb_func cb;
    uint8_t slot_count;
    uint8_t i;

    memset(transmission_pattern, 0, sizeof(transmission_pattern));
    cssm->step_transmission = &transmission_pattern[0];

    prev_step = NULL;
    slot_count = cssm->n_ap;

    for (i = 0; i < sizeof(transmission_pattern); ++i) {
        step = &transmission_pattern[i];

        step->state = ble_ll_cs_proc_next_step_state_get(cssm->step_mode,
                                                         prev_step ? prev_step->state
                                                                   : STEP_STATE_INIT,
                                                         slot_count);
        if (step->state == STEP_STATE_COMPLETE) {
            prev_step->end_tifs = ble_ll_cs_proc_tifs_get(prev_step->state, step->state);
            prev_step->end_transition = BLE_PHY_TRANSITION_NONE;
            break;
        }

        cb = NULL;
        if (step->state == STEP_STATE_CS_TONE_I || step->state == STEP_STATE_CS_TONE_R) {
            if (slot_count == 0) {
                slot_count = cssm->n_ap;

                if ((step->state == STEP_STATE_CS_TONE_I) ? cssm->tone_ext_presence_i
                                                          : cssm->tone_ext_presence_r) {
                    cb = ble_ll_cs_proc_skip_txrx;
                }
            } else {
                --slot_count;
            }
        }

        if (!cb) {
            cb = ble_ll_cs_proc_sched_cb_get(cssm->active_config->role, step->state);
        }

        step->cb = cb;
        step->duration_usecs = ble_ll_cs_proc_step_state_duration_get(step->state, cssm->step_mode,
                                                                      cssm->t_sy, cssm->t_sy_seq);
        step->wfr_usecs = step->duration_usecs;

        if (prev_step) {
            prev_step->end_tifs = ble_ll_cs_proc_tifs_get(prev_step->state, step->state);
            prev_step->end_transition = ble_ll_cs_proc_transition_get(cb);
        }

        prev_step = step;
    }

    return 0;
}

static int
ble_ll_cs_setup_next_step(struct ble_ll_cs_sm *cssm)
{
    int rc;
    const struct ble_ll_cs_config *conf = cssm->active_config;

    cssm->step_anchor_usecs = cssm->anchor_usecs;
    ++cssm->steps_in_procedure_count;
    ++cssm->steps_in_subevent_count;

    rc = ble_ll_cs_proc_subevent_next_state(cssm);
    if (rc) {
        return rc;
    }

    /* Update the transmission anchor, because the step anchor may have been
     * moved to the next subevent.
     */
    cssm->anchor_usecs = cssm->step_anchor_usecs;

    if (cssm->subevent_state != SUBEVENT_STATE_REPETITION_STEP) {
        rc = ble_ll_cs_generate_channel(cssm);
        if (rc) {
            return rc;
        }

        /* Update channel cache used in repetition steps */
        if (cssm->subevent_state == SUBEVENT_STATE_MAINMODE_STEP &&
            conf->main_mode_repetition) {
            cssm->repetition_channels[0] = cssm->repetition_channels[1];
            cssm->repetition_channels[1] = cssm->repetition_channels[2];
            cssm->repetition_channels[2] = cssm->channel;
        }
    } else {
        cssm->channel = cssm->repetition_channels[
            conf->main_mode_repetition - cssm->repetition_count];
    }

    /* Generate CS Access Address */
    rc = ble_ll_cs_drbg_generate_aa(&cssm->drbg_ctx,
                                    cssm->steps_in_procedure_count,
                                    &cssm->initiator_aa, &cssm->reflector_aa);
    if (rc) {
        return rc;
    }

    if (conf->role == BLE_LL_CS_ROLE_INITIATOR) {
        cssm->tx_aa = cssm->initiator_aa;
        cssm->rx_aa = cssm->reflector_aa;
    } else { /* BLE_LL_CS_ROLE_REFLECTOR */
        cssm->rx_aa = cssm->initiator_aa;
        cssm->tx_aa = cssm->reflector_aa;
    }

    rc = ble_ll_cs_drbg_rand_tone_ext_presence(
        &cssm->drbg_ctx, cssm->steps_in_procedure_count, &cssm->tone_ext_presence_i);

    if (rc) {
        return rc;
    }

    rc = ble_ll_cs_drbg_rand_tone_ext_presence(
        &cssm->drbg_ctx, cssm->steps_in_procedure_count, &cssm->tone_ext_presence_r);
    if (rc) {
        return rc;
    }

    ble_ll_cs_proc_trans_pattern_generate(cssm);

    return 0;
}

static int
ble_ll_cs_proc_next_state(struct ble_ll_cs_sm *cssm)
{
    int rc;
    struct ble_ll_cs_step_transmission *step = ++cssm->step_transmission;

    if (step->state == STEP_STATE_COMPLETE) {
        /* Save step results */
        ble_ll_cs_proc_add_step_result(cssm);
    } else if (step->state != STEP_STATE_INIT) {
        /* Continue pending step */
        return 0;
    }

    /* Setup a new step */
    rc = ble_ll_cs_setup_next_step(cssm);
    if (rc) {
        return rc;
    }

    return 0;
}

static int
ble_ll_cs_proc_sched_cb(struct ble_ll_sched_item *sch)
{
    int rc;
    struct ble_ll_cs_sm *cssm = sch->cb_arg;

    BLE_LL_ASSERT(cssm != NULL);

    rc = cssm->sched_cb(cssm);
    if (rc) {
        return BLE_LL_SCHED_STATE_DONE;
    }

    return BLE_LL_SCHED_STATE_RUNNING;
}

static void
ble_ll_cs_proc_mid_transition(struct ble_ll_cs_sm *cssm)
{
    struct ble_ll_cs_step_transmission *step = cssm->step_transmission;

    if (step->end_transition == BLE_PHY_TRANSITION_NONE) {
        ble_phy_transition_set(BLE_PHY_TRANSITION_NONE, 0);
        ble_phy_disable();
        ble_ll_state_set(BLE_LL_STATE_STANDBY);
        cssm->anchor_usecs += step->duration_usecs + step->end_tifs;
    } else {
        ble_phy_transition_set(step->end_transition, step->end_tifs);
    }
}

int
ble_ll_cs_proc_schedule_next_tx_or_rx(struct ble_ll_cs_sm *cssm)
{
    int rc;
    struct ble_ll_cs_step_transmission *step;
    uint32_t anchor_cputime;
    uint8_t transition;
    uint8_t offset;

    if (ble_ll_state_get() == BLE_LL_STATE_CS) {
        ble_ll_cs_proc_mid_transition(cssm);
    }

    rc = ble_ll_cs_proc_next_state(cssm);
    if (rc) {
        return rc;
    }

    step = cssm->step_transmission;
    if (step->cb == ble_ll_cs_sync_rx_start || step->cb == ble_ll_cs_tone_rx_start) {
        /* Start RX windows earlier */
        offset = 2;
        cssm->anchor_usecs -= offset;
    }

    anchor_cputime = ble_ll_tmr_u2t(cssm->anchor_usecs);

    if (anchor_cputime - g_ble_ll_sched_offset_ticks - 1 > ble_ll_tmr_get()) {
        cssm->sch.start_time = anchor_cputime - g_ble_ll_sched_offset_ticks;
        cssm->sched_cb = step->cb;
        cssm->sch.end_time = anchor_cputime + ble_ll_tmr_u2t_up(step->duration_usecs + offset);
        cssm->sch.remainder = 0;
        cssm->sch.sched_type = BLE_LL_SCHED_TYPE_CS;
        cssm->sch.cb_arg = cssm;
        cssm->sch.sched_cb = ble_ll_cs_proc_sched_cb;
        rc = ble_ll_sched_cs_proc(&cssm->sch);
    } else {
        /* Radio start already scheduled, just configure. */
        rc = step->cb(cssm);
    }

    return rc;
}

void
ble_ll_cs_proc_set_now_as_anchor_point(struct ble_ll_cs_sm *cssm)
{
    cssm->anchor_usecs = ble_ll_tmr_t2u(ble_ll_tmr_get());
}

int
ble_ll_cs_proc_scheduling_start(struct ble_ll_conn_sm *connsm, uint8_t config_id)
{
    int rc;
    struct ble_ll_cs_sm *cssm = connsm->cssm;
    struct ble_ll_cs_config *conf;
    const struct ble_ll_cs_proc_params *params;
    uint32_t anchor_ticks;
    uint8_t anchor_rem_usecs;

    conf = &cssm->config[config_id];
    cssm->active_config = conf;
    cssm->active_config_id = config_id;
    params = &conf->proc_params;

    ble_ll_conn_anchor_event_cntr_get(connsm, params->anchor_conn_event_cntr,
                                      &anchor_ticks, &anchor_rem_usecs);

    ble_ll_tmr_add(&anchor_ticks, &anchor_rem_usecs, params->event_offset);

    if (anchor_ticks - g_ble_ll_sched_offset_ticks < ble_ll_tmr_get()) {
        /* The start happend too late for the negotiated event counter. */
        return BLE_ERR_INV_LMP_LL_PARM;
    }

    g_ble_ll_cs_sm_current = cssm;
    cssm->anchor_usecs = ble_ll_tmr_t2u(anchor_ticks);
    cssm->step_mode = BLE_LL_CS_MODE0;
    cssm->n_ap = aci_table[params->aci].n_ap;
    cssm->procedure_anchor_usecs = cssm->anchor_usecs;
    cssm->event_anchor_usecs = cssm->anchor_usecs;
    cssm->subevent_anchor_usecs = cssm->anchor_usecs;
    cssm->step_anchor_usecs = cssm->anchor_usecs;

    cssm->steps_in_procedure_count = ~0;
    cssm->steps_in_subevent_count = ~0;
    cssm->procedure_count = 0;

    memset(transmission_pattern, 0, sizeof(transmission_pattern));
    cssm->step_transmission = &transmission_pattern[0];

    ble_ll_cs_proc_tifs_init(cssm->active_config->t_ip1,
                             cssm->active_config->t_ip2,
                             cssm->t_sw,
                             cssm->active_config->t_fcs);

    ble_ll_cs_proc_calculate_timing(cssm);

    cssm->mode0_next_chan_id = 0xFF;
    cssm->non_mode0_next_chan_id = 0xFF;

    rc = ble_ll_cs_proc_schedule_next_tx_or_rx(cssm);
    if (rc) {
        return BLE_ERR_UNSPECIFIED;
    }

    return BLE_ERR_SUCCESS;
}

void
ble_ll_cs_proc_sync_lost(struct ble_ll_cs_sm *cssm)
{
    ble_ll_cs_proc_set_now_as_anchor_point(cssm);
    ble_phy_transition_set(BLE_PHY_TRANSITION_NONE, 0);
    ble_phy_disable();
    ble_phy_cs_sync_mode_set(0);
    ble_ll_state_set(BLE_LL_STATE_STANDBY);
    /* TODO: Handle a lost sync */
}

/**
 * Called when the wait for response timer expires while in the sync state.
 *
 * Context: Interrupt.
 */
void
ble_ll_cs_proc_wfr_timer_exp(void)
{
    struct ble_ll_cs_sm *cssm = g_ble_ll_cs_sm_current;
    struct ble_ll_cs_step_transmission *step;

    BLE_LL_ASSERT(cssm != NULL);

    step = cssm->step_transmission;
    switch(step->state) {
    case STEP_STATE_CS_SYNC_I:
    case STEP_STATE_CS_SYNC_R:
        ble_ll_cs_proc_sync_lost(cssm);
        break;
    case STEP_STATE_CS_TONE_I:
        if (cssm->active_config->role == BLE_LL_CS_ROLE_REFLECTOR) {
            ble_ll_cs_tone_tx_end_cb(cssm);
        } else {
            ble_ll_cs_tone_rx_end_cb(cssm);
        }
        break;
    case STEP_STATE_CS_TONE_R:
        if (cssm->active_config->role == BLE_LL_CS_ROLE_INITIATOR) {
            ble_ll_cs_tone_rx_end_cb(cssm);
        } else {
            ble_ll_cs_tone_tx_end_cb(cssm);
        }
        break;
    default:
        BLE_LL_ASSERT(0);
    }
}

#endif /* BLE_LL_CHANNEL_SOUNDING */
