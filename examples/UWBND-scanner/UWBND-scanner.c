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
#include "sys/timer.h"
#include "sys/node-id.h"
#include "net/netstack.h"

#include "core/net/linkaddr.h"
/*---------------------------------------------------------------------------*/
PROCESS(range_process, "Test range process");
AUTOSTART_PROCESSES(&range_process);
/*---------------------------------------------------------------------------*/
#define STM32_UUID ((uint32_t *)0x1ffff7e8)
int x = 0;

uint8_t payload[100];
uint8_t rx_msg[100];
uint16_t receivers[20];
int receiver_ind = 0;

dwt_config_t config = {
    3, /* Channel number. */
    DWT_PRF_64M, /* Pulse repetition frequency. */
    DWT_PLEN_4096, /* Preamble length. Used in TX only. */
    DWT_PAC32, /* Preamble acquisition chunk size. Used in RX only. */
    9, /* TX preamble code. Used in TX only. */
    9, /* RX preamble code. Used in RX only. */
    0, /* 0 to use standard SFD, 1 to use non-standard SFD. */
    DWT_BR_6M8, /* Data rate. */
    DWT_PHRMODE_STD, /* PHY header mode. */
    (2000) /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
};

dwt_txconfig_t txConf = {
    0xC9, //PGDelay
    0x18181818 //30 dB
};


uint8_t stop_trans = 0;
clock_time_t start_time;

void tx_ok_cb(const dwt_cb_data_t *cb_data){
  clock_time_t *diff_time = (clock_time_t *) &payload[2];
  clock_time_t current_time = clock_time();
  if (stop_trans == 0){
    *diff_time = current_time - start_time;
    dwt_writetxdata(sizeof(payload), payload, 0);
    dwt_writetxfctrl(sizeof(payload), 0, 0);
    dwt_starttx(DWT_START_TX_IMMEDIATE);
  }else{
    dwt_forcetrxoff();
    dwt_rxenable(DWT_START_RX_IMMEDIATE);
    // printf("TX OK %d\n", *cnt);
  }
}

void rx_ok_cb(const dwt_cb_data_t *cb_data){
  dwt_readrxdata(rx_msg, cb_data->datalength, 0);
  uint8_t *node = (uint8_t *) &rx_msg[2];
  dwt_forcetrxoff();
  dwt_rxreset();
  dwt_rxenable(DWT_START_RX_IMMEDIATE);
  receivers[receiver_ind++] = *node;
}


void rx_err_cb(const dwt_cb_data_t *cb_data){
  
  dwt_forcetrxoff();
  dwt_rxenable(DWT_START_RX_IMMEDIATE);
  printf("RX ERR: %x\n", cb_data->status);
  // printf("TX OK Sender\n");
}





PROCESS_THREAD(range_process, ev, data)
{
  static struct etimer et;
  static struct etimer timeout;
  static int status;

  PROCESS_BEGIN();
  static struct etimer et;
  printf("STARTING scanner with PLEN %d, payload size: %d\n", config.txPreambLength, sizeof(payload));
  dwt_setcallbacks(&tx_ok_cb, &rx_ok_cb, NULL, &rx_err_cb);
  clock_init();
  etimer_set(&et, CLOCK_SECOND * 9);
  PROCESS_WAIT_UNTIL(etimer_expired(&et));
  dwt_configure(&config);
  dwt_configuretxrf(&txConf);
  dwt_forcetrxoff();
  payload[0] = 0xff;
  payload[1] = 0xa;
  dwt_writetxdata(sizeof(payload), payload, 0);
  dwt_writetxfctrl(sizeof(payload), 0, 0);

  while (1){
    
    stop_trans = 0;
    dwt_forcetrxoff();
    start_time = clock_time();
    printf("Start sending WaK %d, %d\n", payload[1], start_time);
    dwt_writetxfctrl(sizeof(payload), 0, 0);
    dwt_starttx(DWT_START_TX_IMMEDIATE);
    etimer_set(&et, 110); // TX frame for 110 ms
    PROCESS_WAIT_UNTIL(etimer_expired(&et));
    stop_trans = 1; // Once done TX start RX
    dwt_setpreambledetecttimeout(0);
    printf("TX done\n");
    dwt_forcetrxoff();
    dwt_rxreset();
    dwt_rxenable(DWT_START_RX_IMMEDIATE);
    etimer_set(&et, 2000);
    PROCESS_WAIT_UNTIL(etimer_expired(&et));
    printf("Report %d -> ", receiver_ind);
    for (int i=0; i < receiver_ind; i++){
      printf("%d, ", receivers[i]);
    }
    printf("\n");
    receiver_ind = 0;
    dwt_forcetrxoff(); // Finish Scanning and wait for 30s
    // break;
    etimer_set(&et, CLOCK_SECOND);
    PROCESS_WAIT_UNTIL(etimer_expired(&et));
    payload[1]++;
    /* code */
  }
  
  PROCESS_END();
}
/*---------------------------------------------------------------------------*/