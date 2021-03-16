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

#ifndef ATPC_H
#define ATPC_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    ATPC_BEACON_IND,
    ATPC_BEACON_RSP,
    ATPC_QUALITY_MONITOR,
    ATPC_NOTIFICATION,
} atpc_msg_type_t;

typedef struct {
    atpc_msg_type_t type;
    int8_t power_level;
    int8_t rssi;
} __attribute__((packed, aligned(1))) atpc_msg_t;

typedef struct {
    uint16_t src_addr;
    int8_t rssi;
    atpc_msg_t *data;
} atpc_data_ind_t;

/*!
 * @brief   ATPC custom send message function pointer definition
 *
 * @param   dest_addr - destination's short identification address
 *
 * @param   tx_power - transmission power in the unit used by the platform.
 *
 * @param   data - pointer to payload
 *
 * @param   len - payload length
 *
 * @return  True if successful, false otherwise.
 */
typedef bool (*atpc_send_msg_t)(uint16_t dest_addr, int8_t tx_power,
                                   uint8_t *data, uint16_t len);

/*!
 * @brief   Linked list new list function pointer definition.
 *
 * @return  pointer to the list that is being created.
 */
typedef void *(*atpc_list_new_t)(void);

/*!
 * @brief   Linked list insert item function pointer definition.
 *
 * @param   list - pointer to linked list.
 *
 * @param   new_item - pointer to item to be inserted in the list.
 */
typedef void (*atpc_list_add_t)(void *list, void *item);

/*!
 * @brief   Linked list remove item function pointer definition.
 *
 * @param   list - pointer to linked list.
 *
 * @param   item - pointer to item to be removed from the list.
 */
typedef void (*atpc_list_remove_t)(void *list, void *item);

/*!
 * @brief   Linked list retrieve next item function pointer definition.
 *
 * @param   list - pointer to linked list.
 *
 * @return  pointer to the next item.
 */
typedef void *(*atpc_list_next_t)(void *list);

/*!
 * @brief   Get the current time.
 *
 * @return  current time in microseconds.
 */
typedef uint64_t (*atpc_get_time_t)(void);

typedef void (*atpc_log_t)(const char *aFormat, ...);

/* ATPC callbacks structure */
typedef struct {
    atpc_send_msg_t send_msg;
    atpc_list_new_t list_new;
    atpc_list_add_t list_add;
    atpc_list_remove_t list_remove;
    atpc_list_next_t list_next;
    atpc_get_time_t get_time;
    atpc_log_t log;
} atpc_callbacks_t;

/*!
 * @brief   Configure and start ATPC providing the necessary callbacks and
 *          transmission information.
 *
 * @param   callbacks - pointer to structure containing custom ATPC functions
 *          provided by external application.
 * @param   default_tx_power - value of default transmit power level.
 * @param   tx_power - pointer to array of possible transmit power values in the
 *          unit used by the platform.
 * @param   tx_power_count - number of possible transmit power values.
 * @param   rssi_setpoint - RSSI setpoint for the control model. 
 * @param   rssi_threshold_upper - upper value of RSSI threshold range for the
 *          control model. 
 * @param   rssi_threshold_lower - lower value of RSSI threshold range for the
 *          control model.
 * @param   multicast_addr - address to send multicast beacons.
 * @param   beacon_timeout - maximum time to wait for a beacon response (in
 *          microseconds.
 */
void atpc_conf( atpc_callbacks_t *callbacks,
                int8_t default_tx_power,
                int8_t *tx_power,
                uint8_t tx_power_count,
                int8_t rssi_setpoint,
                int8_t rssi_threshold_upper,
                int8_t rssi_threshold_lower,
                uint16_t multicast_addr,
                uint64_t beacon_timeout );

/*!
 * @brief   Initialize ATPC beacons. Must be called after atpc_conf and may not 
 *          be necessary in case of a full function device (FFD) that only
 *          monitors messages from reduced function devices (RFDs), replying
 *          with ATPC notifications. Application should manage the timing of the
 *          initialization of devices.
 */
void atpc_init(void);

/*!
 * @brief   Process ATPC states. Must be called after atpc_init by external 
 *          application periodically or during every relevant event in order to
 *          update models.
 */
void atpc_process(void);

/*!
 * @brief   Register new neighbor.
 *
 * @param   short_addr - neighbor's short identification address
 *
 * @return  status
 */
int8_t atpc_register_neighbor(uint16_t short_addr);

/*!
 * @brief   Remove neighbor.
 *
 * @param   short_addr - neighbor's short identification address
 *
 * @return  status
 */
int8_t atpc_remove_neighbor(uint16_t short_addr);

/*!
 * @brief   Get transmit power level. Called by external application when
 *          sending messages.
 *
 * @param   short_addr - destination's short identification address
 *
 * @return  Transmit power level
 */
int8_t atpc_get_tx_power(uint16_t short_addr);

/*!
 * @brief   Data indication callback. Called by external application when it
 *          receives an ATPC message.
 *
 * @param   
 */
void atpc_data_ind(atpc_data_ind_t *ind);

#ifdef __cplusplus
} // extern "C"
#endif

#endif /* ATPC_H */

