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
#include "nrf.h"
#include "core/net/linkaddr.h"
/*---------------------------------------------------------------------------*/
PROCESS(range_process, "Test range process");
AUTOSTART_PROCESSES(&range_process);
/*---------------------------------------------------------------------------*/
uint16_t node_id;
/*---------------------------------------------------------------------------*/

uint16_t get_node_addr(){
  uint16_t node_id = 0;
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


typedef enum{
  RX_WAK_P1 = 0,
  RX_WAK_P2 = 1,
  RX_WAK_P3 = 2,
  WAITING_TS = 3,
  WAITING = 4,
  PKT_DETECTED=5,
  CCA = 6,
  RDY_TO_TX = 7,
  WAIT_FOR_RESP = 8,
}DETECTION_STATUS;
/*---------------------------------------------------------------------------*/
#define STM32_UUID ((uint32_t *)0x1ffff7e8)
#define PDTO                            3
#define SFD_TO                          1
#define PAC                             DWT_PAC8
#define SNIFF_INTERVAL                  500
#define RAPID_SNIFF_INTERVAL            250
#define P2_TO_THRESH                    SNIFF_INTERVAL + 10
#define CLS_TO_THRESH                   500
#define CCA_EN                          1
#define TS_MODE                         0

#define WAC2_PC                         1

/*---------------------------------------------------------------------------*/
dwt_config_t config = {
    1, /* Channel number. */
    DWT_PRF_64M, /* Pulse repetition frequency. */
    DWT_PLEN_256, /* Preamble length. Used in TX only. */
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
int P2_timeout = 0;
int CLS_timeout = 0;
bool config_change = false;
int cca_timer = 0;
int back_off = 0;
int tot_sniffs = 0;
int wac1_sniffs, wac2_sniffs;
int tot_wac1_scan = 0;

clock_time_t wac1_sniff_time, wac2_sniff_time, tx_time, diff_time;

int wac1_sniff_interval = SNIFF_INTERVAL;
/*---------------------------------------------------------------------------*/

void tx_ok_cb(const dwt_cb_data_t *cb_data){
}

void rx_ok_cb(const dwt_cb_data_t *cb_data){
  dwt_readrxdata(rx_payload, cb_data->datalength, 0);
  dwt_forcetrxoff();
  dwt_rxreset();
  switch (detection_status){
  case RX_WAK_P1:
    detection_status = RX_WAK_P2;
    break;
  case RX_WAK_P2:
    detection_status = CCA;
    break;

  case CCA:
    detection_status = PKT_DETECTED;
    break;
  
  case WAITING_TS:
    if(rx_payload[0] == 0xAA){
      printf("Received TS MSG\n");
      detection_status = WAITING;
    }else{
      dwt_rxenable(DWT_START_RX_IMMEDIATE);
    }
    break; 

  default:
    printf("Something received\n");
    detection_status = RX_WAK_P1;
    break;
  }
  
}


void rx_err_cb(const dwt_cb_data_t *cb_data){
  dwt_forcetrxoff();
  dwt_rxreset();
  switch (detection_status){
  case RX_WAK_P1:
    detection_status = RX_WAK_P2;
    printf("Detected WaC1: %d, %d, %d\n", node_id, wac1_sniffs, tot_wac1_scan);
    wac1_sniffs = 0;
    break;

  case RX_WAK_P2:
    detection_status = RX_WAK_P2;

  case WAITING_TS:
    dwt_rxenable(DWT_START_RX_IMMEDIATE);
    break;
  case WAIT_FOR_RESP:
    printf("Sensend RESP but got ERR\n");
    detection_status = RX_WAK_P1;
    break;
    
  default:
    detection_status = RX_WAK_P2;
    break;
  }
}


void rx_to_cb(const dwt_cb_data_t *cb_data){
  dwt_forcetrxoff();
  switch (detection_status){
  case RX_WAK_P2:
    detection_status = RDY_TO_TX;
    break;
  default:
    detection_status = RX_WAK_P1;
    break;
  }
}
/*---------------------------------------------------------------------------*/

PROCESS_THREAD(range_process, ev, data)
{
  static struct etimer et;
  static struct etimer timeout;
  static int status;
  static int sniff_cnt = 0;

  PROCESS_BEGIN();
  static struct etimer et;
  dwt_setcallbacks(&tx_ok_cb, &rx_ok_cb, &rx_to_cb, &rx_err_cb);
  node_id = get_node_addr();
  dwt_configure(&config);
  dwt_configuretxrf(&txConf);
  clock_init();
  dwt_forcetrxoff();
  dwt_setpreambledetecttimeout(PDTO);  
  payload[2] = node_id;
  memcpy(&payload[2], (uint16_t *) &node_id, 2);
  detection_status = RX_WAK_P1;
  

  while (1){
    etimer_set(&et, 1);
    PROCESS_WAIT_UNTIL(etimer_expired(&et));
    if (detection_status == RX_WAK_P1){
      config_change = false;
      if (config.rxCode != 9){
        config_change = true;
        config.rxCode = 9;
        config.prf = DWT_PRF_64M;
      }
      if (config.rxPAC != PAC){
        config_change = true;
        config.rxPAC = PAC;
      }
      if (config.sfdTO != SFD_TO){
        config_change = true;
        config.sfdTO = SFD_TO;
      }
      if (config_change){
        dwt_configure(&config);
      }
      dwt_forcetrxoff();
      dwt_rxreset();
      etimer_set(&et, wac1_sniff_interval - 1);

      P2_timeout = 0;
      CLS_timeout = 0;
      back_off = 0;
      sniff_cnt = 1;
      tot_sniffs++;
      wac1_sniffs++;
      tot_wac1_scan++;
      wac1_sniff_interval = SNIFF_INTERVAL;
      
      PROCESS_WAIT_UNTIL(etimer_expired(&et));
      dwt_rxenable(DWT_START_RX_IMMEDIATE);
      wac1_sniff_time = clock_time();
      // printf("WaC1 time %d\n", wac1_sniff_time);

    }
    if (detection_status == RX_WAK_P2){
      dwt_forcetrxoff();
      dwt_rxreset();
      etimer_set(&et, RAPID_SNIFF_INTERVAL - 1);
      PROCESS_WAIT_UNTIL(etimer_expired(&et));
      dwt_rxenable(DWT_START_RX_IMMEDIATE);
      wac2_sniff_time = clock_time();
      // printf("WaC2 time %d\n", wac2_sniff_time - wac1_sniff_time);
      sniff_cnt += 1;
      tot_sniffs++;
    }
    if (detection_status == RDY_TO_TX){
      
      config.rxPAC = PAC;
      config.prf = DWT_PRF_64M;
      config.rxCode = 9;
      config.sfdTO = 300;
      dwt_configure(&config);
      dwt_forcetrxoff();
      dwt_writetxdata(sizeof(payload), payload, 0);
      dwt_writetxfctrl(sizeof(payload), 0, 0);
      printf("TX: %d, %d, %d\n", node_id, sniff_cnt, tot_sniffs);
      tx_time = clock_time();
      // printf("TX time: %d\n", tx_time - wac1_sniff_time);
      if(dwt_starttx(DWT_START_TX_IMMEDIATE) != DWT_SUCCESS){
        printf("TX ERR\n");
      }
      diff_time = SNIFF_INTERVAL - (tx_time - wac1_sniff_time);
      printf("diff time %d\n", diff_time);
      if (diff_time < 15 || diff_time > SNIFF_INTERVAL)
        diff_time = SNIFF_INTERVAL;
      wac1_sniff_interval = diff_time - 11;
      printf("diff time %d\n", diff_time);
      etimer_set(&et, 5);
      PROCESS_WAIT_UNTIL(etimer_expired(&et));
      dwt_forcetrxoff();
      dwt_setpreambledetecttimeout(PDTO);
      detection_status = WAIT_FOR_RESP;
      dwt_rxenable(DWT_START_RX_IMMEDIATE);
      etimer_set(&et, 5);
      PROCESS_WAIT_UNTIL(etimer_expired(&et));
    }

  }
  
  PROCESS_END();
}
/*---------------------------------------------------------------------------*/