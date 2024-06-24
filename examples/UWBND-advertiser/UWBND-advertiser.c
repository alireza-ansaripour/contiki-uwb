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
  RX_WAK_P3 = 4,
  WAITING = 5,
  CCA = 2,
  RDY_TO_TX = 3,
}DETECTION_STATUS;
/*---------------------------------------------------------------------------*/
#define STM32_UUID ((uint32_t *)0x1ffff7e8)
#define PDTO       3
#define SFD_TO     1
#define PAC   DWT_PAC8
#define SNIFF_INTERVAL 100
#define RAPID_SNIFF_INTERVAL 25
#define WAC_TO  130
#define P2_TO_THRESH     500
#define CLS_TO_THRESH     500
/*---------------------------------------------------------------------------*/
dwt_config_t config = {
    3, /* Channel number. */
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

uint8_t payload[] = {'a', 'l', 0, 0, 0, 0};
uint8_t rx_payload[20];
DETECTION_STATUS detection_status = RX_WAK_P1;
int WaC_wating = 0;
int back_off = 0;
int P2_timeout = 0;
int CLS_timeout = 0;
bool config_change = false;
int sniff_counter = 0;
int false_wake_up_cnt = 0;
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

  default:
    break;
  }
  
}


void rx_err_cb(const dwt_cb_data_t *cb_data){
  dwt_forcetrxoff();
  dwt_rxreset();
  switch (detection_status){
  case RX_WAK_P1:
    detection_status = RX_WAK_P1;
    false_wake_up_cnt++;
    printf("Detected WaC1: %d, %d, %d\n", node_id, sniff_counter, false_wake_up_cnt);
    
    sniff_counter = 0;
    break;
  case RX_WAK_P2:
    detection_status = WAITING;
    printf("Detected WaC2: %d\n", node_id);
    break;
  
  case CCA:
    detection_status = CCA;
    break;
  default:
    break;
  }
}


void rx_to_cb(const dwt_cb_data_t *cb_data){
  dwt_forcetrxoff();
  switch (detection_status){
  case RX_WAK_P2:
    detection_status = RX_WAK_P2;
    break;
  case CCA:
    printf("TX %d\n", node_id);
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
  

  PROCESS_BEGIN();
  static struct etimer et;
  dwt_setcallbacks(&tx_ok_cb, &rx_ok_cb, &rx_to_cb, &rx_err_cb);
  if(deployment_set_node_id_ieee_addr()){
    printf("NODE addr set successfully: %d\n", node_id);
  }else{
    printf("Failed to set nodeID\n");
  }
  printf("This is the new version \n");
  // deployment_print_id_info();
  etimer_set(&et, CLOCK_SECOND * 1);
  dwt_configure(&config);
  PROCESS_WAIT_UNTIL(etimer_expired(&et));
  dwt_forcetrxoff();
  dwt_setpreambledetecttimeout(PDTO);  
  payload[2] = node_id;
  memcpy(&payload[2], (uint8_t *) &node_id, 2);
  detection_status = RX_WAK_P1;
  sniff_counter = 0;
  false_wake_up_cnt = 0;  

  while (1){
    etimer_set(&et, 1);
    PROCESS_WAIT_UNTIL(etimer_expired(&et));
    if (detection_status == RX_WAK_P1){
      etimer_set(&et, SNIFF_INTERVAL - 1);
      PROCESS_WAIT_UNTIL(etimer_expired(&et));
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
      dwt_rxenable(DWT_START_RX_IMMEDIATE);
      P2_timeout = 0;
      CLS_timeout = 0;
      sniff_counter++;
    }
    if (detection_status == RX_WAK_P2){
      config_change = false;
      if (config.rxCode != 5){
        config_change = true;
        config.rxCode = 5;
        config.prf = DWT_PRF_16M;
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
      etimer_set(&et, RAPID_SNIFF_INTERVAL - 1);
      PROCESS_WAIT_UNTIL(etimer_expired(&et));
      P2_timeout += RAPID_SNIFF_INTERVAL;
      if (P2_timeout >= P2_TO_THRESH){
        dwt_forcetrxoff();
        dwt_rxreset();
        detection_status = RX_WAK_P1;
      }
      dwt_forcetrxoff();
      dwt_rxreset();
      dwt_rxenable(DWT_START_RX_IMMEDIATE);
    }
    if (detection_status == WAITING){
      config.prf = DWT_PRF_64M;
      config.rxCode = 9;
      config.rxPAC = DWT_PAC32;
      config.sfdTO = 8000;
      dwt_configure(&config);
      unsigned short r = random_rand() % 20;
      dwt_setpreambledetecttimeout(200 + (r * 30));
      etimer_set(&et, RAPID_SNIFF_INTERVAL - 1);
      PROCESS_WAIT_UNTIL(etimer_expired(&et));
      detection_status = CCA;
      dwt_rxenable(DWT_START_RX_IMMEDIATE);
      printf("Starting CCA\n");
    }

    if (detection_status == CCA){
      dwt_rxenable(DWT_START_RX_IMMEDIATE);
      etimer_set(&et, 2);
      PROCESS_WAIT_UNTIL(etimer_expired(&et));
    }
    if (detection_status == RDY_TO_TX){
      detection_status = RX_WAK_P1;
      config.rxPAC = PAC;
      config.prf = DWT_PRF_64M;
      config.rxCode = 9;
      config.sfdTO = SFD_TO;
      dwt_configure(&config);
      dwt_forcetrxoff();
      dwt_writetxdata(sizeof(payload), payload, 0);
      dwt_writetxfctrl(sizeof(payload), 0, 0);
      if(dwt_starttx(DWT_START_TX_IMMEDIATE) != DWT_SUCCESS){
        printf("TX ERR\n");

        printf("oh no\n");
      }
      etimer_set(&et, 200);
      PROCESS_WAIT_UNTIL(etimer_expired(&et));
      dwt_forcetrxoff();
      dwt_setpreambledetecttimeout(PDTO);
    }

  }
  
  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
