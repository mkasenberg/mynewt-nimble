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
#include "controller/ble_ll_conn.h"
#include "controller/ble_ll_sched.h"
#include "controller/ble_ll_tmr.h"
#include "ble_ll_cs_priv.h"

struct ble_ll_cs_sm *g_ble_ll_cs_sm_current;

#define SUBEVENT_STATE_MODE0_STEP      (0)
#define SUBEVENT_STATE_REPETITION_STEP (1)
#define SUBEVENT_STATE_SUBMODE_STEP    (2)
#define SUBEVENT_STATE_MAINMODE_STEP   (3)

static struct ble_ll_cs_aci aci_table[] = {
    {1, 1, 1}, {2, 2, 1}, {3, 3, 1}, {4, 4, 1},
    {2, 1, 2}, {3, 1, 3}, {4, 1, 4}, {4, 2, 2}
};

/* A pattern containing the states and transitions of the current step */
static struct ble_ll_cs_step_transmission transmission_pattern[12];

void ble_ll_cs_tone_tx_end_cb(struct ble_ll_cs_sm *cssm);
void ble_ll_cs_tone_rx_end_cb(struct ble_ll_cs_sm *cssm);

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
    /* TODO: Setup new CS step */

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

    memset(transmission_pattern, 0, sizeof(transmission_pattern));
    cssm->step_transmission = &transmission_pattern[0];

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
