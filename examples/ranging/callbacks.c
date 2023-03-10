#include "callbacks.h"
#include "deca_device_api.h"
#include <stdio.h>

// asdfasdf;
void tx_ok_cb(const dwt_cb_data_t *cb_data){
    printf("Sender callback: TX_OK\n");
}