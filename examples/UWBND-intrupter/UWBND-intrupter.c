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
#include "sys/node-id.h"
#include "net/netstack.h"

#include "core/net/linkaddr.h"
#include <sys/node-id.h>
/*---------------------------------------------------------------------------*/
PROCESS(range_process, "Test range process");
AUTOSTART_PROCESSES(&range_process);


uint16_t get_node_addr(){
  uint16_t node_id;
  uint32_t dev_id = NRF_FICR->DEVICEADDR[0];
  switch (dev_id){
    case 0x5270f477:
      node_id = 166;
      break;

    case 0x752a0381:
      node_id = 161;
      break;
    
    case 0x81984018:
      node_id = 165;
      break;
    
    case 0x5c50e9de:
      node_id = 168;
      break;
    
    case 0xaaf5c764:
      node_id = 162;
      break;
    
    case 0x4ed6a168:
      node_id = 170;
      break;

    case 0x25571c0e:
      node_id = 167;
      break;

    case 0x723ee061:
      node_id = 163;
      break;
    
    case 0xda82e887:
      node_id = 169;
      break;

    case 0x7605ae4e:
      node_id = 173;
      break;
    

    case 0x2510ed2a:
      node_id = 172;
      break;
    
    case 0x685c382a:
      node_id = 164;
      break;

    case 0xabe717f8:
      node_id = 171;
      break;
  };

  return node_id;

}


/*---------------------------------------------------------------------------*/
#define STM32_UUID ((uint32_t *)0x1ffff7e8)
#define TX_INTERVAL 12

uint8_t payload[3];
uint16_t node_id;
int tx_cnt = 0;

dwt_config_t config = {
    1, /* Channel number. */
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
static int index_cnt = 0;


struct Scan_report{
  int index;
  uint16_t ids[20];
};

static struct Scan_report report;
clock_time_t start_time, end_time;



void tx_ok_cb(const dwt_cb_data_t *cb_data){
  tx_cnt ++;
}

void rx_ok_cb(const dwt_cb_data_t *cb_data){
  dwt_forcetrxoff();
  dwt_rxenable(DWT_START_RX_IMMEDIATE);
  dwt_readrxdata(payload, cb_data->datalength, 0);
  report.ids[index_cnt++] = payload[2];
}


void rx_err_cb(const dwt_cb_data_t *cb_data){
  dwt_forcetrxoff();
  dwt_rxenable(DWT_START_RX_IMMEDIATE);
  // printf("TX OK Sender\n");
}



uint16_t wait, wait2;


PROCESS_THREAD(range_process, ev, data)
{
  static struct etimer et;
  static struct etimer timeout;
  static int status;
  int res;
  PROCESS_BEGIN();
  static struct etimer et;
  dwt_setcallbacks(&tx_ok_cb, &rx_ok_cb, NULL, &rx_err_cb);


  clock_init();
  node_id = get_node_addr();
  switch(node_id){
    case 58:
    case 13:
    case 166:
      config.prf = DWT_PRF_16M;
      config.txCode = 1;
    break;

  }
  dwt_configure(&config);
  dwt_configuretxrf(&txConf);
  dwt_forcetrxoff();

  dwt_writetxdata(sizeof(payload), payload, 0);
  dwt_writetxfctrl(sizeof(payload), 0, 0);
  printf("Starting Interupter %d\n", TX_INTERVAL);
  dwt_setpreambledetecttimeout(0);
  index_cnt = 0;

  while (1){
    index_cnt = 0;
    stop_trans = 0;
    dwt_forcetrxoff();
    start_time = clock_time();
    wait = (random_rand() % (TX_INTERVAL));
    wait2 = TX_INTERVAL - wait;
    
    etimer_set(&et, wait); // wait for some time after the transmission
    PROCESS_WAIT_UNTIL(etimer_expired(&et));
    dwt_writetxfctrl(sizeof(payload), 0, 0);
    res = dwt_starttx(DWT_START_TX_IMMEDIATE);
    etimer_set(&et, wait2); // wait for some time after the transmission
    PROCESS_WAIT_UNTIL(etimer_expired(&et));
    end_time = clock_time();
    dwt_forcetrxoff();
    
  }
  


  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
