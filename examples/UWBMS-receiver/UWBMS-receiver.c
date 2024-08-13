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
#define UUS_TO_DWT_TIME 65536
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

typedef struct{
  uint8_t rxCode;
  uint8_t ts_rxCode;
  uint16_t rx_wait_us;
} rx_info_t;


/*---------------------------------------------------------------------------*/
packet_t rxpkt;

dwt_config_t config =   {
    5, /* Channel number. */
    DWT_PRF_64M, /* Pulse repetition frequency. */
    DWT_PLEN_1024, /* Preamble length. Used in TX only. */
    DWT_PAC16, /* Preamble acquisition chunk size. Used in RX only. */
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
rx_info_t rx_info;
/*---------------------------------------------------------------------------*/

void rx_ok_cb(const dwt_cb_data_t *cb_data){
  dwt_readrxdata((uint8_t *) &rxpkt, 20, 0);
  uint32_t rx_timestamp;
  uint32_t tx_time;
  rx_timestamp = dwt_readrxtimestamphi32();
  dwt_forcetrxoff();
  
  // if (rxpkt.seq - last_seq != 1)
  //   printf("Diff: %d, %d\n", rxpkt.seq, rxpkt.seq - last_seq);
  if (rxpkt.packet_type == 1){
    config.rxCode = rx_info.rxCode;
    dwt_configure(&config);
    dwt_setpreambledetecttimeout(500);
    tx_time = rx_timestamp + ((UUS_TO_DWT_TIME * rx_info.rx_wait_us) >> 8);
    dwt_setdelayedtrxtime(tx_time);
    dwt_rxenable(DWT_START_RX_DELAYED);
  }
  
  if (rxpkt.packet_type == 2){
    last_seq = rxpkt.seq;
    packets_received++;
    config.rxCode = rx_info.ts_rxCode;
    dwt_configure(&config);
    dwt_rxenable(DWT_START_RX_IMMEDIATE);
    dwt_setpreambledetecttimeout(0);
  }
  
}

void rx_to_cb(const dwt_cb_data_t *cb_data){
  config.rxCode = rx_info.ts_rxCode;
  dwt_configure(&config);
  dwt_setpreambledetecttimeout(0);
  dwt_rxenable(DWT_START_RX_IMMEDIATE);
}

void rx_err_cb(const dwt_cb_data_t *cb_data){
  dwt_forcetrxoff();
  config.rxCode = rx_info.ts_rxCode;
  dwt_configure(&config);
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

  dwt_setcallbacks(NULL, &rx_ok_cb, &rx_to_cb, &rx_err_cb);

  printf("NODE ID is: %d\n", node_id);
  rx_info.ts_rxCode = config.rxCode;

  switch (node_id){    
    case 50:
        rx_info.rxCode = 10;
        rx_info.rx_wait_us = 1200;
      break;
    case 51:
        rx_info.rxCode = 11;
        rx_info.rx_wait_us = 1800;
      break;
    case 52:
        rx_info.rxCode = 12;
        rx_info.rx_wait_us = 1500;
      break;
    case 53:
        rx_info.rxCode = 13;
        rx_info.rx_wait_us = 2100;
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
