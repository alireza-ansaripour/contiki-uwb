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
#include <sys/node-id.h>
#include "net/netstack.h"
#include "core/net/linkaddr.h"
/*---------------------------------------------------------------------------*/
PROCESS(range_process, "Test range process");
AUTOSTART_PROCESSES(&range_process);
/*---------------------------------------------------------------------------*/
typedef struct {
  uint8_t packet_type;
  uint16_t src;
  uint16_t dst;
  uint32_t seq;
  uint8_t payload[1200];
} packet_t;

typedef struct{
  uint8_t sender_id;
  uint8_t config_id;
} time_sync_payload;

typedef struct{
  uint8_t sender_id;
  uint32_t ts_seq;
} data_payload;

/*---------------------------------------------------------------------------*/
packet_t rxpkt;

dwt_config_t config =   {
    5, /* Channel number. */
    DWT_PRF_64M, /* Pulse repetition frequency. */
    DWT_PLEN_1024, /* Preamble length. Used in TX only. */
    DWT_PAC8, /* Preamble acquisition chunk size. Used in RX only. */
    9, /* TX preamble code. Used in TX only. */
    9, /* RX preamble code. Used in RX only. */
    1, /* 0 to use standard SFD, 1 to use non-standard SFD. */
    DWT_BR_6M8, /* Data rate. */
    DWT_PHRMODE_EXT, /* PHY header mode. */
    (8000 + 1 + 64 - 64) /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
};
int bytes_received = 0;

uint32_t last_seq = 0;
uint32_t start_seq = 0;

int packets_received = 0;
/*---------------------------------------------------------------------------*/

void rx_ok_cb(const dwt_cb_data_t *cb_data){
  dwt_readrxdata((uint8_t *) &rxpkt, 20, 0);
  dwt_forcetrxoff();
  dwt_rxenable(DWT_START_RX_IMMEDIATE);
  // if (rxpkt.seq - last_seq != 1)
  //   printf("Diff: %d, %d\n", rxpkt.seq, rxpkt.seq - last_seq);
  last_seq = rxpkt.seq;
  packets_received++;
}

void rx_err_cb(const dwt_cb_data_t *cb_data){
  dwt_forcetrxoff();
  dwt_rxenable(DWT_START_RX_IMMEDIATE);
  // printf("RX ERR CB receiver: %d\n", cb_data->status);
}



PROCESS_THREAD(range_process, ev, data)
{
  static struct etimer et;

  PROCESS_BEGIN();
  
  if(deployment_set_node_id_ieee_addr()){
    printf("NODE addr set successfully: %d\n", node_id);
  }else{
    printf("Failed to set nodeID\n");
  }
  dwt_setcallbacks(NULL, &rx_ok_cb, NULL, &rx_err_cb);


  switch (node_id){
    case 126:
        config.rxCode = 11;
      break;

    case 128:
        config.rxCode = 12;
      break;

    case 131:
        config.rxCode = 9;
      break;
    
    case 135:
        config.rxCode = 10;
      break;
    
    default:
      break;
  }

  dwt_configure(&config);
  bytes_received = 0;
  dwt_forcetrxoff();
  dwt_rxenable(DWT_START_RX_IMMEDIATE);
  while (1){
    etimer_set(&et, CLOCK_SECOND);
    PROCESS_WAIT_UNTIL(etimer_expired(&et));
    if (last_seq != 0){
      printf("PRR: %d\n", (100 * packets_received) / (last_seq - start_seq));
      start_seq = last_seq;
      packets_received = 0;
    }else{
      printf("No data yet!!!\n");
    }
  }
  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
