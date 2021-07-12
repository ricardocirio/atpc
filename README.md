# ATPC: Adaptive Transmission Power Control

## About

A generic implementation of the ATPC algorithm for wireless sensor networks
based on the work published by 
[Lin et al. (2006)](https://dl.acm.org/doi/10.1145/1182807.1182830). 
Developed at [IFSC](https://www.ifsc.edu.br), this implementation was written
generically in order to be compatible with different protocols and platforms. 
It was tested on an nRF52840 as an OpenThread network device in a controlled 
setup, obtaining a 43 % reduction in the transmission energy, compared to the 
use of a fixed transmission power. In addition, there was no decrease in the 
packet reception rate.

## How to use

1. Configure and start module by calling *atpc_conf*, providing the necessary 
callbacks and transmission information. Set up your application in order to 
call *atpc_data_ind* when it receives a message, passing the proper data 
structure containing information about the received message. Register 
neighbors within the ATPC module using *atpc_register_neighbor*. Examples for 
*atpc_conf* and *atpc_data_ind* below:

    - *atpc_conf*:
    atpc_conf(&atpc_cbs,
        DEFAULT_TX_POWER, TX_POWER, sizeof(TX_POWER)/sizeof(int8_t),
        RSSI_SETPOINT, RSSI_UPPER_THRESH, RSSI_LOWER_THRESH,
        MULTICAST_ADDR , BEACON_TIMEOUT);

    - *atpc_data_ind*:
    static void handle_receive(uint16_t src_addr, int8_t rssi,
                               void *buff, uint8_t length) {
        atpc_data_ind_t data_ind;
        atpc_msg_t msg;
        data_ind.src_addr = src_addr;
        data_ind.rssi = rssi;
        switch(buff[0]) {
            case APP_MSG:
                msg.type = ATPC_QUALITY_MONITOR;
                data_ind.data = &msg;
                break;
            case ATPC_MSG:
                data_ind.data = (atpc_msg_t *)(buff + sizeof(app_msg_type_t));
                break;
            default:
                break;
        }
        atpc_data_ind(&data_ind);
    }

3. Initialize ATPC beacons by calling *atpc_init*. This may not be necessary in
case of a full function device (FFD) that only monitors messages from reduced 
function devices (RFDs), replying with ATPC notifications. Application should 
manage the timing of the initialization of devices appropriately.

4. Execute internal state machine with *atpc_process* - call it periodically 
until initialization is completed, then call it to update the control model 
of neighbors whenever it may be necessary. Keep your neighbor list updated 
through *atpc_register_neighbor* and *atpc_remove_neighbor* as needed. Use 
*atpc_get_tx_power* to obtain the adaptive transmission power before sending 
each message. Application is responsible for managing the scope in which each 
function is called, employing mutual exclusion or semaphores if necessary.

## Author

[Ricardo Schons Cirio](https://www.linkedin.com/in/ricardocirio/) (Electronics 
Engineer).

