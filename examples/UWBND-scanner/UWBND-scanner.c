/*
 * Copyright (c) 2017, University of Trento.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials provided
 *    with the distribution.
 * 3. The name of the author may not be used to endorse or promote
 *    products derived from this software without specific prior
 *    written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include "contiki.h"
#include "lib/random.h"
#include "net/rime/rime.h"
#include "leds.h"
#include "net/netstack.h"
#include <stdio.h>
#include "dw1000.h"
#include "dw1000-ranging.h"
#include <string.h>
#include <inttypes.h>
#include <stdio.h>
#include <math.h>
// #include "sys/node-id.h"
// #include "net/netstack.h"

// #include "core/net/linkaddr.h"
#include <sys/node-id.h>


/*---------------------------------------------------------------------------*/
PROCESS(range_process, "Test range process");
AUTOSTART_PROCESSES(&range_process);
#define UUS_TO_DWT_TIME     65536


struct Scan_report{
  int index;
  uint16_t ids[20];
};

typedef enum{
  CCA_1,
  CCA_2,
  WAK_1,
  WAK_2,
  SEND_RPLY,
  LISTENING,
}DETECTION_STATUS;

/*---------------------------------------------------------------------------*/

#define SCAN_INTERVAL      1000
#define SCAN_DURATION      500
/*---------------------------------------------------------------------------*/

uint8_t payload[10];
uint8_t msg[7] = {0xbe, 0, 0, 0, 0, 0, 0};

dwt_config_t config = {
    5, /* Channel number. */
    DWT_PRF_64M, /* Pulse repetition frequency. */
    DWT_PLEN_4096, /* Preamble length. Used in TX only. */
    DWT_PAC8, /* Preamble acquisition chunk size. Used in RX only. */
    9, /* TX preamble code. Used in TX only. */
    9, /* RX preamble code. Used in RX only. */
    0, /* 0 to use standard SFD, 1 to use non-standard SFD. */
    DWT_BR_6M8, /* Data rate. */
    DWT_PHRMODE_STD, /* PHY header mode. */
    (8000) /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
};


dwt_txconfig_t txConf = {
    0xC9, //PGDelay
    0x18181818 //30 dB
};


/*----------------------------------------------------------------------------------*/

void tx_ok_cb(const dwt_cb_data_t *cb_data){
  
}

void rx_ok_cb(const dwt_cb_data_t *cb_data){
  dwt_rxenable(DWT_START_RX_IMMEDIATE);
  printf("RX OK");
}


void rx_err_cb(const dwt_cb_data_t *cb_data){
  dwt_forcetrxoff();
  dwt_rxenable(DWT_START_RX_IMMEDIATE);
  printf("RX ERR");
}


/*-------------------------------------------------------------------*/

int counter = 0;
uint32_t status_reg;
PROCESS_THREAD(range_process, ev, data)
{
  static struct etimer et;
  static struct etimer timeout;
  static int status;

  PROCESS_BEGIN();
  static struct etimer et;
  dw1000_set_isr(dwt_isr);
  dwt_setinterrupt(DWT_INT_TFRS | DWT_INT_RFCG | DWT_INT_RFTO | DWT_INT_RXPTO |
                  DWT_INT_RPHE | DWT_INT_RFCE | DWT_INT_RFSL | DWT_INT_SFDT |
                  DWT_INT_ARFE, 1);

  dwt_setcallbacks(&tx_ok_cb, &rx_ok_cb, &rx_err_cb, &rx_err_cb);
  etimer_set(&et, 100);
  PROCESS_WAIT_UNTIL(etimer_expired(&et));


  if(deployment_set_node_id_ieee_addr()){
    printf("NODE addr set successfully: %d\n", node_id);
  }else{
    printf("Failed to set nodeID\n");
  }

  dwt_configure(&config);
  dwt_configuretxrf(&txConf);
  dwt_forcetrxoff();
  clock_init();
  dwt_writetxdata(sizeof(msg), msg, 0);
  dwt_writetxfctrl(sizeof(msg), 0, 0);
  printf("Starting scanner:%d \n", node_id);
  dwt_setpreambledetecttimeout(0);
  dwt_rxenable(DWT_START_RX_IMMEDIATE);
  while (1){
    
    // dwt_forcetrxoff();
    // dwt_rxenable(DWT_START_RX_IMMEDIATE);
    // etimer_set(&et, SCAN_DURATION);
    // PROCESS_WAIT_UNTIL(etimer_expired(&et));
    // dwt_forcetrxoff();
    etimer_set(&et, SCAN_INTERVAL -  SCAN_DURATION);
    PROCESS_WAIT_UNTIL(etimer_expired(&et));
    

    
  }
  


  PROCESS_END();
}
/*---------------------------------------------------------------------------*/