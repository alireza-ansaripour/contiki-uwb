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

#define WaC1_LEN_MS      505
#define WaC2_LEN_MS      60   
#define LISTEN_LEN_MS    65
#define TS_MSG           0

#define WAC2_PC          1
#define SCAN_INTERVAL    100

/*---------------------------------------------------------------------------*/

uint8_t payload[10];
uint8_t msg[7] = {0xbe, 0, 0, 0, 0, 0, 0};
uint8_t stop_trans = 0;
static int index_cnt = 0;
static int error_cnt = 0;
static struct Scan_report report;
DETECTION_STATUS detection_status = CCA_1;
uint32_t WaC_start_time, WaC_current_time;
clock_time_t scan_init_time, scan_end_time;

clock_time_t listen_begin_time, listen_end_time;
uint32_t adv_rx_time, rep_tx_time;

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
    (8000) /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
};

dwt_txconfig_t txConf = {
    0xC9, //PGDelay
    0x18181818 //30 dB
};


/*----------------------------------------------------------------------------------*/

void tx_ok_cb(const dwt_cb_data_t *cb_data){
  if (stop_trans == 0){
    WaC_current_time = clock_time();
    uint32_t *time_diff = (uint32_t *) &msg[1];
    *time_diff = WaC_current_time - WaC_start_time;
    dwt_writetxdata(sizeof(msg), msg, 0);
    dwt_writetxfctrl(sizeof(msg), 0, 0);
    dwt_starttx(DWT_START_TX_IMMEDIATE);
  }
}

void rx_ok_cb(const dwt_cb_data_t *cb_data){
  dwt_forcetrxoff();
  dwt_rxenable(DWT_START_RX_IMMEDIATE);
  dwt_readrxdata(payload, cb_data->datalength, 0);
  if (payload[0] == 0xad){
    adv_rx_time = dwt_readrxtimestamphi32();
    uint16_t *n_id = (uint16_t *) &payload[2];
    report.ids[index_cnt++] = *n_id;
    // detection_status = SEND_RPLY;
  }
}


void rx_err_cb(const dwt_cb_data_t *cb_data){
  dwt_forcetrxoff();
  dwt_rxenable(DWT_START_RX_IMMEDIATE);
  error_cnt++;
  // printf("TX OK Sender\n");
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
  etimer_set(&et, CLOCK_SECOND * 2);
  PROCESS_WAIT_UNTIL(etimer_expired(&et));
  dwt_configure(&config);
  dwt_configuretxrf(&txConf);
  dwt_forcetrxoff();
   clock_init();
  dwt_writetxdata(sizeof(msg), msg, 0);
  dwt_writetxfctrl(sizeof(msg), 0, 0);
  printf("Starting scanner with WaC22: %d, SCAN_INTERVAL %d\n", WaC2_LEN_MS, SCAN_INTERVAL);
  dwt_setpreambledetecttimeout(0);
  index_cnt = 0;

  while (1){
    scan_init_time = clock_time();    
    printf("Start sending WaK: %d\n", scan_init_time);
    /* ------------------------ Sending WaC1 --------------------------------*/
    stop_trans = 0;
    dwt_forcetrxoff();
    memset((uint8_t *) &msg[1], 0, sizeof(msg) - 1);
    dwt_writetxdata(sizeof(msg), msg, 0);
    dwt_writetxfctrl(sizeof(msg), 0, 0);
    detection_status = WAK_1;
    dwt_starttx(DWT_START_TX_IMMEDIATE);
    WaC_start_time = clock_time();
    etimer_set(&et, WaC1_LEN_MS); // TX WaC1
    PROCESS_WAIT_UNTIL(etimer_expired(&et));
    stop_trans = 1; 
    dwt_forcetrxoff();
    dwt_rxreset();
    index_cnt = 0;
    
    /* ----------------------- Changing to WaC2 -------------------------------------*/
    printf("changing config\n");
    config.prf = DWT_PRF_16M;
    config.txCode = WAC2_PC;
    dwt_configure(&config);
    dwt_writetxdata(sizeof(msg), msg, 0);
    dwt_writetxfctrl(sizeof(msg), 0, 0);
    stop_trans = 0;
    detection_status = WAK_2;
    dwt_starttx(DWT_START_TX_IMMEDIATE);
    etimer_set(&et, WaC2_LEN_MS); // TX WaC2
    PROCESS_WAIT_UNTIL(etimer_expired(&et));
    stop_trans = 1; // Once done TX start RX
    /*------------------------------------------------------------------------------*/
    dwt_forcetrxoff();
    config.prf = DWT_PRF_64M;
    config.txCode = 9;
    dwt_configure(&config);
    error_cnt = 0;
    dwt_forcetrxoff();
    listen_begin_time = clock_time();
    detection_status = LISTENING;

    ;
    printf("Listening %d\n", dwt_rxenable(DWT_START_RX_IMMEDIATE) == DWT_SUCCESS);
    

    while (detection_status == LISTENING){
      etimer_set(&et, 1);
      PROCESS_WAIT_UNTIL(etimer_expired(&et));  
      listen_end_time = clock_time();
      if (listen_end_time - listen_begin_time >= LISTEN_LEN_MS){
        printf("LISTEN TO\n");
        break;
      }
    }

    etimer_set(&et, 3);
    PROCESS_WAIT_UNTIL(etimer_expired(&et));
    
    msg[0] = 0xEF;
    dwt_forcetrxoff();
    if (detection_status == SEND_RPLY){
      dwt_writetxdata(sizeof(msg), msg, 0);
      dwt_writetxfctrl(sizeof(msg), 0, 0);
      // rep_tx_time = adv_rx_time + (4000 * UUS_TO_DWT_TIME) >> 8;
      // dwt_setdelayedtrxtime(rep_tx_time);
      printf("sending REP %d\n", dwt_starttx(DWT_START_TX_IMMEDIATE) == DWT_SUCCESS);
    }
    
    etimer_set(&et, 6);
    PROCESS_WAIT_UNTIL(etimer_expired(&et));
    

    scan_end_time = clock_time();
    printf("Report T:%d, %d: %d -> ", detection_status, scan_end_time - scan_init_time, index_cnt);
    for (int i = 0; i < index_cnt; i++){
      printf("%d, ", report.ids[i]);
    }
    printf("\n");
    printf("Error cnt: %d\n", error_cnt);
    

    etimer_set(&et, (SCAN_INTERVAL * 1000) - (scan_end_time - scan_init_time));
    PROCESS_WAIT_UNTIL(etimer_expired(&et));
  }
  


  PROCESS_END();
}
/*---------------------------------------------------------------------------*/