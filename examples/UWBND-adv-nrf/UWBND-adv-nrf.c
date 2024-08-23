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
#include "sys/node-id.h"
#include "nrf.h"
#include <sys/node-id.h>
#include "core/net/linkaddr.h"
/*---------------------------------------------------------------------------*/
PROCESS(range_process, "Test range process");
AUTOSTART_PROCESSES(&range_process);
#define PDTO       3

/*---------------------------------------------------------------------------*/

uint16_t get_node_ddr(){
  uint32_t dev_id = NRF_FICR->DEVICEADDR[0];
  uint16_t node_id = 1;
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
    
    default:
      node_id = 160;
      break;
    
  };
  return node_id;

}




typedef enum{
  RX_WAK_P1 = 0,
  RX_WAK_P2 = 1,
  CLS_TO_TX = 2,
  RDY_TO_TX = 3,
}DETECTION_STATUS;


uint8_t payload[] = {'a', 'l', 0, 0, 0, 0};
uint8_t status;
uint8_t rx_msg[100];
DETECTION_STATUS detection_status = RX_WAK_P1;
int wait_time = 10;
uint16_t pre_count;


dwt_txconfig_t txConf = {
    0xC9, //PGDelay
    0x18181818 //30 dB
};

void tx_ok_cb(const dwt_cb_data_t *cb_data){
}

uint8_t last_adv_id = 0;
void rx_ok_cb(const dwt_cb_data_t *cb_data){
  dwt_rxdiag_t diag_info;
  dwt_readdiagnostics(&diag_info);
  dwt_readrxdata(rx_msg, cb_data->datalength, 0);
  dwt_forcetrxoff();
  dwt_rxreset();
  if(rx_msg[0] != 0xff){
    return;
  }
  uint8_t msg_adv_id = rx_msg[1];
  if (msg_adv_id != last_adv_id){
    clock_time_t *time = (clock_time_t *) &rx_msg[2];
    status = 1;
    printf("ADV:detected WaC: %d, %d, %d\n", diag_info.pacNonsat,msg_adv_id, *time);
    last_adv_id = msg_adv_id;
    
  }
}


void rx_err_cb(const dwt_cb_data_t *cb_data){
  dwt_rxreset();
  dwt_forcetrxoff();
  printf("RX ERR\n");
}


void rx_to_cb(const dwt_cb_data_t *cb_data){
  dwt_rxreset();
  dwt_forcetrxoff();
}

dwt_config_t config = {
    3, /* Channel number. */
    DWT_PRF_64M, /* Pulse repetition frequency. */
    DWT_PLEN_2048, /* Preamble length. Used in TX only. */
    DWT_PAC8, /* Preamble acquisition chunk size. Used in RX only. */
    9, /* TX preamble code. Used in TX only. */
    9, /* RX preamble code. Used in RX only. */
    0, /* 0 to use standard SFD, 1 to use non-standard SFD. */
    DWT_BR_6M8, /* Data rate. */
    DWT_PHRMODE_STD, /* PHY header mode. */
    (8000) /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
};









PROCESS_THREAD(range_process, ev, data)
{
  static struct etimer et;
  static struct etimer timeout;
  uint16_t node_id;

  PROCESS_BEGIN();
  node_id = get_node_ddr();
  etimer_set(&et, CLOCK_SECOND * 1);
  dwt_configure(&config);
  dwt_configuretxrf(&txConf);
  dwt_setsmarttxpower(0);
  PROCESS_WAIT_UNTIL(etimer_expired(&et));
  dwt_forcetrxoff();
  memcpy(&payload[2], (uint8_t *) &node_id, 1);
  status = 0;
  dwt_setpreambledetecttimeout(PDTO);
  while (1){
    etimer_set(&et, 1);
    PROCESS_WAIT_UNTIL(etimer_expired(&et));
    if (status == 0){
      dwt_rxenable(DWT_START_RX_IMMEDIATE);
      etimer_set(&et, 99);
      PROCESS_WAIT_UNTIL(etimer_expired(&et));
    }
    if (status == 1){
      clock_time_t *time = (clock_time_t *) &rx_msg[2];
      int sleep_time = 110 - *time + ((node_id % 100) * 2);
      etimer_set(&et, sleep_time);
      PROCESS_WAIT_UNTIL(etimer_expired(&et));
      dwt_writetxdata(sizeof(payload), payload, 0);
      dwt_writetxfctrl(sizeof(payload), 0, 0);
      dwt_starttx(DWT_START_TX_IMMEDIATE);
      printf("TX %d\n", node_id);
      status = 0;
    }
    
  }
  
  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
