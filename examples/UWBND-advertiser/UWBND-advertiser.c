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
#include <sys/node-id.h>
#include "core/net/linkaddr.h"
/*---------------------------------------------------------------------------*/
PROCESS(range_process, "Test range process");
AUTOSTART_PROCESSES(&range_process);
/*---------------------------------------------------------------------------*/
typedef enum{
  RX_WAK_P1 = 0,
  RX_WAK_P2 = 1,
  RX_WAK_P3 = 2,
  WAITING_TS = 3,
  WAITING = 4,
  PKT_DETECTED=5,
  CCA = 6,
  RDY_TO_TX = 7,
}DETECTION_STATUS;
/*---------------------------------------------------------------------------*/
#define STM32_UUID ((uint32_t *)0x1ffff7e8)
#define PDTO                            3
#define SFD_TO                          1
#define PAC                             DWT_PAC8
#define SNIFF_INTERVAL                  500
#define RAPID_SNIFF_INTERVAL            50
#define WAC_TO                          130
#define P2_TO_THRESH                    SNIFF_INTERVAL + 10
#define CLS_TO_THRESH                   500
#define CCA_EN                          0
#define TS_MODE                         0
/*---------------------------------------------------------------------------*/
dwt_config_t config = {
    3, /* Channel number. */
    DWT_PRF_64M, /* Pulse repetition frequency. */
    DWT_PLEN_64, /* Preamble length. Used in TX only. */
    PAC, /* Preamble acquisition chunk size. Used in RX only. */
    9, /* TX preamble code. Used in TX only. */
    9, /* RX preamble code. Used in RX only. */
    0, /* 0 to use standard SFD, 1 to use non-standard SFD. */
    DWT_BR_6M8, /* Data rate. */
    DWT_PHRMODE_STD, /* PHY header mode. */
    (SFD_TO) /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
};


dwt_txconfig_t txConf = {
    0xC9, //PGDelay
    0x18181818 //30 dB
};

uint8_t payload[] = {0xad, 0, 0, 0, 0, 0};
uint8_t rx_payload[20];
DETECTION_STATUS detection_status = RX_WAK_P1;
int WaC_wating = 0;
int back_off = 0;
int P2_timeout = 0;
int CLS_timeout = 0;
bool config_change = false;
/*---------------------------------------------------------------------------*/

void tx_ok_cb(const dwt_cb_data_t *cb_data){
}

void rx_ok_cb(const dwt_cb_data_t *cb_data){
  dwt_readrxdata(rx_payload, cb_data->datalength, 0);
  dwt_forcetrxoff();
  dwt_rxreset();
  dwt_rxenable(DWT_START_RX_IMMEDIATE);  
}


void rx_err_cb(const dwt_cb_data_t *cb_data){
  dwt_forcetrxoff();
  dwt_rxreset();
  dwt_rxenable(DWT_START_RX_IMMEDIATE);
}


void rx_to_cb(const dwt_cb_data_t *cb_data){
  dwt_forcetrxoff();
}
/*---------------------------------------------------------------------------*/

PROCESS_THREAD(range_process, ev, data)
{
  static struct etimer et;
  static struct etimer timeout;
  static int status;
  static int T_ADV;
  

  PROCESS_BEGIN();
  static struct etimer et;
  dwt_setcallbacks(&tx_ok_cb, &rx_ok_cb, &rx_to_cb, &rx_err_cb);
  if(deployment_set_node_id_ieee_addr()){
    printf("NODE addr set successfully: %d\n", node_id);
  }else{
    printf("Failed to set nodeID\n");
  }
  
  
  T_ADV = 100;
  dwt_configure(&config);
  dwt_configuretxrf(&txConf);
  dwt_forcetrxoff(); 
  memcpy(&payload[2], (uint16_t *) &node_id, 2);
  dwt_writetxdata(sizeof(payload), (uint8_t *) payload, 0);
  printf("Start advertiser wit T_ADV: %d\n", T_ADV);
  while (1){
    etimer_set(&et, T_ADV);
    PROCESS_WAIT_UNTIL(etimer_expired(&et));
    dwt_writetxfctrl(sizeof(payload), 0, 0);
    dwt_starttx(DWT_START_TX_IMMEDIATE);
    printf("ADV sent\n");
  }
  
  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
