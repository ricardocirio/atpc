/******************************************************************************
 *   ATPC: Adaptive Transmission Power Control.
 *   Copyright (C) 2021  Ricardo Cirio (INSTITUTO FEDERAL DE SANTA CATARINA)
 *
 *   This program is free software: you can redistribute it and/or modify
 *   it under the terms of the GNU Lesser General Public License as published by
 *   the Free Software Foundation, either version 3 of the License, or
 *   any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU Lesser General Public License for more details.
 *
 *   You should have received a copy of the GNU Lesser General Public License
 *   along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *****************************************************************************/

#include "atpc.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/******************************************************************************
 Constants and definitions
 *****************************************************************************/

/* Number of bits of fractional part of Qm.n format numbers */
#define Q   3 // Q.3 format number

/* Constant used for rounding fractional numbers */
#define K (1 << (Q - 1))

/*
 * Control model structure containing the constants of an approximated function
 * that relates the RSSI and the transmission power level.
 *
 *  r = a*tp + b
 *
 *  Where
 *      r: RSSI
 *      tp: transmission power level
 */
typedef struct {
    /* Q number format */
    int16_t a;
    /* Q number format */
    int16_t b;
} control_model_t;

/* Neighbor information structure */
typedef struct {
    /* Neighbor's short identification address */
    uint16_t short_addr;
    /* Neighbor status: 0 for initializing, 1 for runtime */
    bool status;
    /* Update model flag: set to 1 when control model needs to be updated */
    bool update_model;
    /* Indicate that there is an initialization beacon in progress */
    bool beacon_active;
    /* Status of each possible transmission power distributed as a bit field
     * vector of size multiple of the uint8_t size. */
    uint8_t *tp_status;
    /* Variable to keep track of initialization beacons. Indicates the index of
       the current beacon being processed for a neighbor, corresponding to the
       transmit power value array. */
    uint8_t beacon_count;
    /* RSSI vector for values measured by neighbor corresponding to the beacon
       sent at each transmission power */
    int8_t *rssi;
    /* Difference between setpoint and latest RSSI received in a notification
       beacon from neighbor */
    int8_t delta_rssi;
    /* Transmit power control model */
    control_model_t control_model;
    /* Timestamp of when last beacon was sent (in microseconds) */
    uint64_t beacon_timestamp;
} atpc_neighbor_t;

/* ATPC data structure */
typedef struct {
    /* List of atpc_neighbor_t items containing neighbors information */
    void *list;
    /* Keep track of neighbor count */
    uint8_t neighbor_count;
    /* ATPC callbacks */
    atpc_callbacks_t *cb;
    /* Message buffer */
    atpc_msg_t msg;
    /* Default transmit power level */
    int8_t default_tx_power;
    /* Pointer to array of possible transmit power values */
    int8_t *tx_power;
    /* Number of elements in tx_power array */
    uint8_t tx_power_count;
    /* RSSI setpoint for the control model */
    int8_t rssi_setpoint;
    /* Upper value of RSSI threshold range for the control model */
    int8_t rssi_threshold_upper;
    /* Lower value of RSSI threshold range for the control model */
    int8_t rssi_threshold_lower;
    /* Address to send multicast beacons */
    uint16_t multicast_addr;
    /* Maximum time to wait for a beacon response */
    uint64_t beacon_timeout;
} atpc_data_t;

/* FSM states */
typedef enum {
    STATE_IDLE,
    STATE_INIT,
    STATE_RUN,
    NUM_STATES
} state_t;

/* Definition of FSM state function vector */
typedef struct {
    state_t state;
    void (*func)(void);
} fsm_t;

/******************************************************************************
 Global variables
 *****************************************************************************/

/******************************************************************************
 Local function prototypes
 *****************************************************************************/

static atpc_neighbor_t *find_neighbor(uint16_t short_addr);
static int8_t find_tp_idx(int8_t tp);
static int32_t q_mul(int32_t a, int32_t b);
static int32_t q_div(int32_t a, int32_t b);
static void state_idle(void);
static void state_init(void);
static void state_run(void);

/******************************************************************************
 Local variables
 *****************************************************************************/

static atpc_data_t atpc;

static fsm_t fsm[] = {
    {STATE_IDLE, state_idle},
    {STATE_INIT, state_init},
    {STATE_RUN, state_run}
};

static state_t state = STATE_IDLE;

/******************************************************************************
 Public Functions
 *****************************************************************************/

void atpc_conf( atpc_callbacks_t *callbacks,
                int8_t default_tx_power,
                int8_t *tx_power,
                uint8_t tx_power_count,
                int8_t rssi_setpoint,
                int8_t rssi_threshold_upper,
                int8_t rssi_threshold_lower,
                uint16_t multicast_addr,
                uint64_t beacon_timeout ) {
    memset(&atpc, 0, sizeof(atpc_data_t));
    atpc.cb = callbacks;
    atpc.default_tx_power = default_tx_power;
    atpc.tx_power = tx_power;
    atpc.tx_power_count = tx_power_count;
    atpc.rssi_setpoint = rssi_setpoint;
    atpc.rssi_threshold_upper = rssi_threshold_upper;
    atpc.rssi_threshold_lower = rssi_threshold_lower;
    atpc.multicast_addr = multicast_addr;
    atpc.beacon_timeout = beacon_timeout;
    atpc.list = atpc.cb->list_new();
}

void atpc_init(void) {
    state = STATE_INIT;
}

void atpc_process(void) {
    (*fsm[state].func)();
}

int8_t atpc_register_neighbor(uint16_t short_addr) {
    if(find_neighbor(short_addr) != NULL) {
        // already registered
        return -1;
    }

    atpc_neighbor_t *neighbor = malloc(sizeof(atpc_neighbor_t)); 
    memset(neighbor, 0, sizeof(atpc_neighbor_t));

    neighbor->rssi = malloc(sizeof(int8_t)*(atpc.tx_power_count));
    memset(neighbor->rssi, 0, sizeof(int8_t)*(atpc.tx_power_count));

    neighbor->tp_status = malloc((atpc.tx_power_count + 7) / 8);
    memset(neighbor->tp_status, 0, (atpc.tx_power_count + 7) / 8);

    neighbor->short_addr = short_addr;
    atpc.cb->list_add(atpc.list, (void *)neighbor);
    atpc.neighbor_count++;
    return 1;
}

int8_t atpc_remove_neighbor(uint16_t short_addr) {
    atpc_neighbor_t *neighbor = find_neighbor(short_addr);

    if(neighbor == NULL) {
        return -1;
    }

    atpc.cb->list_remove(atpc.list, (void *)neighbor);
    free(neighbor->rssi);
    free(neighbor->tp_status);
    free(neighbor);

    return 1;
}

int8_t atpc_get_tx_power(uint16_t short_addr) {
    int8_t res = atpc.default_tx_power;

    atpc_neighbor_t *neighbor = find_neighbor(short_addr);

    if(neighbor != NULL) {
        if(neighbor->status) {
            /* Calculate desired transmission power */
            int8_t tp = (uint8_t)(q_div(((int16_t)atpc.rssi_setpoint << Q) -
                                            neighbor->control_model.b,
                                            neighbor->control_model.a) >> Q);

            /* Default to maximum */
            res = atpc.tx_power[atpc.tx_power_count - 1];
            
            /* Compare with possible values */
            for(uint8_t i = 0; i < atpc.tx_power_count; i++) {
                if(tp <= atpc.tx_power[i]) {
                    res = atpc.tx_power[i];
                    break;
                } 
            }
        }
    }

    return res;
}

void atpc_data_ind(atpc_data_ind_t *ind) {
    atpc_neighbor_t *neighbor = find_neighbor(ind->src_addr);

    if(neighbor == NULL) {
        /* Neighbor not found */
        return;
    }

    /* Process ATPC message */
    switch(ind->data->type) {
        case ATPC_BEACON_IND: {
            atpc.cb->log("[atpc] BEACON_IND: src_addr: %d, "
                        "power level: %d dBm, rssi: %d dBm",
                        ind->src_addr, ind->data->power_level, ind->rssi);
            atpc.msg.type = ATPC_BEACON_RSP;
            atpc.msg.power_level = ind->data->power_level;
            atpc.msg.rssi = ind->rssi;
            atpc.cb->send_msg(neighbor->short_addr, atpc.default_tx_power,
                              (uint8_t *)&atpc.msg, sizeof(atpc_msg_t));
            break;
        }
        
        case ATPC_BEACON_RSP: {
            atpc.cb->log("[atpc] BEACON_RSP: src_addr: %d, "
                        "power level %d dBm, rssi %d dBm", ind->src_addr,
                        ind->data->power_level, ind->data->rssi);
            int8_t tp_idx = find_tp_idx(ind->data->power_level);
            if(tp_idx >= 0) {
                neighbor->rssi[tp_idx] = ind->data->rssi;
                neighbor->tp_status[tp_idx / 8] |= (1 << (tp_idx % 8));
            }
            neighbor->beacon_active = false;
            neighbor->beacon_count++;
            break;
        }

        case ATPC_QUALITY_MONITOR: {
            if(ind->rssi > atpc.rssi_threshold_upper ||
               ind->rssi < atpc.rssi_threshold_lower) {
                atpc.cb->log("[atpc] QUALITY_MONITOR: src_addr: %d, "
                        "rssi %d dBm", ind->src_addr, ind->rssi);
                atpc.msg.type = ATPC_NOTIFICATION;
                atpc.msg.rssi = atpc.rssi_setpoint - ind->rssi;
                atpc.cb->send_msg(neighbor->short_addr,
                                  atpc_get_tx_power(ind->src_addr),
                                  (uint8_t *)&atpc.msg, sizeof(atpc_msg_t));
            } 
            break;
        }

        case ATPC_NOTIFICATION: {
            atpc.cb->log("[atpc] NOTIFICATION: src_addr: %d, "
                        "delta_rssi %d dBm", ind->src_addr, ind->data->rssi);
            neighbor->delta_rssi = ind->data->rssi;
            neighbor->update_model = true;
            break;
        }

        default: {
            break;
        }
    }
}

/******************************************************************************
 Local Functions
 *****************************************************************************/

static atpc_neighbor_t *find_neighbor(uint16_t short_addr) {
    for(size_t i = 0; i < atpc.neighbor_count; i++) {
        atpc_neighbor_t *neighbor =
                            (atpc_neighbor_t *)atpc.cb->list_next(atpc.list);
        if(neighbor->short_addr == short_addr) {
            return neighbor;
        }
    }
    return NULL;
}

static int8_t find_tp_idx(int8_t tp) {
    for(uint8_t i = 0; i < atpc.tx_power_count; i++) {
        if(tp == atpc.tx_power[i]) {
            return (int8_t)i;
        }
    }
    return -1;
}

static int32_t q_mul(int32_t a, int32_t b) {
    return ((a * b + K) >> Q);
}

static int32_t q_div(int32_t a, int32_t b) {
    int32_t temp = a << Q;
    if((temp >= 0 && b >= 0) || (temp < 0 && b < 0)) {
        temp += (b >> 1);
    } else {
        temp -= (b >> 1);
    }
    return (temp / b);
}

/******************************************************************************
 State Machine Functions
 *****************************************************************************/

static void state_idle(void) {

}

static void state_init(void) {
    for(uint8_t i = 0; i < atpc.neighbor_count; i++) {
        atpc_neighbor_t *neighbor = 
                            (atpc_neighbor_t *)atpc.cb->list_next(atpc.list);

        if(neighbor->beacon_active == true) {
            if((atpc.cb->get_time() - neighbor->beacon_timestamp) <
                atpc.beacon_timeout) {
                /* Keep waiting for beacon response */
                return;
            } else {
                /* Timeout, proceed to next beacon */
                atpc.cb->log("[atpc] Beacon response timeout.");
                neighbor->beacon_active = false;
                neighbor->beacon_count++;
            }
        }

        if(neighbor->status == false) {
            if(neighbor->beacon_count < atpc.tx_power_count) {
                atpc.cb->log("[atpc] sending beacon %d, power level %d dBm",
                            neighbor->beacon_count,
                            atpc.tx_power[neighbor->beacon_count]);
                atpc.msg.type = ATPC_BEACON_IND;
                atpc.msg.power_level = atpc.tx_power[neighbor->beacon_count];
                atpc.cb->send_msg(neighbor->short_addr,
                                  atpc.tx_power[neighbor->beacon_count],
                                  (uint8_t *)&atpc.msg, sizeof(atpc_msg_t));
                neighbor->beacon_timestamp = atpc.cb->get_time();
                neighbor->beacon_active = true;
                state = STATE_INIT;
            } else if(neighbor->beacon_active == false) {
                state = STATE_RUN;
            }
        }
    }}
}

static void state_run(void) {
    for(uint8_t i = 0; i < atpc.neighbor_count; i++) {
        atpc_neighbor_t *neighbor = 
                            (atpc_neighbor_t *)atpc.cb->list_next(atpc.list);

        if(neighbor->status == false) {
            /* Calculate control model coeficients */
            uint8_t num_tp = 0;
            int32_t sum_tp_squared = 0, sum_tp = 0, sum_ri = 0, sum_tp_ri = 0;

            for(uint8_t j = 0; j < atpc.tx_power_count; j++) {
                if((neighbor->tp_status[j / 8]) & (1 << (j % 8))) {
                    sum_tp_squared += atpc.tx_power[j] * atpc.tx_power[j];
                    sum_tp += atpc.tx_power[j];
                    sum_ri += neighbor->rssi[j];
                    sum_tp_ri += atpc.tx_power[j] * neighbor->rssi[j];
                    num_tp++;
                }
            }

            int32_t denominator =   q_mul(num_tp << Q, sum_tp_squared << Q) -
                                    q_mul(sum_tp << Q, sum_tp << Q);

            neighbor->control_model.a =
                            q_div(  q_mul(num_tp << Q, sum_tp_ri << Q) -
                                    q_mul(sum_tp << Q, sum_ri << Q),
                                    denominator );

            neighbor->control_model.b =
                            q_div(  q_mul(sum_ri << Q, sum_tp_squared << Q) -
                                    q_mul(sum_tp << Q, sum_tp_ri << Q),
                                    denominator );

            atpc.cb->log("[atpc] Control model: "
                        "sum_tp_squared = %d, sum_tp = %d, sum_ri = %d, "
                        "sum_tp_ri: %d, a = %d/8, b = %d/8",
                        sum_tp_squared, sum_tp, sum_ri, sum_tp_ri,
                        neighbor->control_model.a, neighbor->control_model.b);
            
            neighbor->status = true;
        } else if(neighbor->update_model) {
            // TODO:    Add support for piggyback notifications, because in
            //          the current implementation the model is updated with
            //          each notification.

            /* Update control model */
            neighbor->control_model.b -= (int16_t)neighbor->delta_rssi << Q; 

            neighbor->update_model = false;

            atpc.cb->log("[atpc] Model updated: b = %d/8",
                            neighbor->control_model.b);
        }
    }

    state = STATE_RUN;
}

