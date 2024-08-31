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


struct Scan_report{
  int index;
  uint16_t ids[20];
};

typedef enum{
  CCA_1,
  CCA_2,
  WAK_1,
  WAK_2,
  TS,
  LISTEN
}DETECTION_STATUS;

/*---------------------------------------------------------------------------*/

#define WaC1_LEN_MS      505
#define WaC2_LEN_MS      52
#define LISTEN_LEN_MS    100
#define TS_MSG           0
#define SNIFF_ON_TIME  1
#define SNIFF_OFF_TIME 48
/*---------------------------------------------------------------------------*/

uint8_t payload[10];
uint8_t msg[7] = {0xbe, 0, 0, 0, 0, 0, 0};
uint8_t stop_trans = 0;
static int index_cnt = 0;
static struct Scan_report report;
DETECTION_STATUS detection_status = CCA_1;
uint32_t WaC_start_time, WaC_current_time;
int T_SCAN, T_INT;

dwt_config_t config = {
    3, /* Channel number. */
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

PROCESS(send_adv, "Test range process");
PROCESS_THREAD(send_adv, ev, data){
  PROCESS_BEGIN();
  printf("send adv message\n");
  dwt_forcetrxoff();
  PROCESS_END();
}


void rx_ok_cb(const dwt_cb_data_t *cb_data){
  dwt_forcetrxoff();
  dwt_rxenable(DWT_START_RX_IMMEDIATE);
  dwt_readrxdata(payload, cb_data->datalength, 0);
  if (payload[0] == 0xad){

    uint16_t *n_id = (uint16_t *) &payload[2];
    bool add = true;
    for (int i = 0; i < index_cnt; i++){
      if (report.ids[i] == *n_id)
        add = false;
    }
    if (add){
      report.ids[index_cnt++] = *n_id;
      printf("found %d\n", *n_id);
      process_start(&send_adv, NULL);
    }
      
  }
}


void rx_err_cb(const dwt_cb_data_t *cb_data){
  dwt_forcetrxoff();
  dwt_rxenable(DWT_START_RX_IMMEDIATE);
  printf("ERR\n");
}


/*-------------------------------------------------------------------*/


PROCESS_THREAD(range_process, ev, data)
{
  static struct etimer et;
  static struct etimer timeout;
  static int status;

  PROCESS_BEGIN();
  static struct etimer et;
  dwt_setcallbacks(&tx_ok_cb, &rx_ok_cb, NULL, &rx_err_cb);
  etimer_set(&et, CLOCK_SECOND * 9);
  PROCESS_WAIT_UNTIL(etimer_expired(&et));
  dwt_configure(&config);
  dwt_configuretxrf(&txConf);
  dwt_forcetrxoff();

  T_SCAN = 500;
  T_INT  = 1000;

  printf("Starting Scanner with: T_ADV = %d, & T_INT = %d\n", T_SCAN, T_INT);

  while (1){
    index_cnt = 0;
    dwt_rxenable(DWT_START_RX_IMMEDIATE);
    etimer_set(&et, T_SCAN);
    PROCESS_WAIT_UNTIL(etimer_expired(&et));    
    dwt_forcetrxoff();
    printf("b'Report: %d -> ", index_cnt);
    for (int i =0 ; i< index_cnt; i++){
      printf("%d, ", report.ids[i]);
    }
    printf("\n");
    etimer_set(&et, T_INT - T_SCAN);
    PROCESS_WAIT_UNTIL(etimer_expired(&et));    
  }
  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
