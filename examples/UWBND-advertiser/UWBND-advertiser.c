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
PROCESS(report_stat, "Test range process");
AUTOSTART_PROCESSES(&range_process, &report_stat);
#define UUS_TO_DWT_TIME     65536
/*---------------------------------------------------------------------------*/
typedef enum{
  RX_WAK_P1 = 0,
  RX_WAK_P2 = 1,
  RX_WAK_P3 = 2,
  WAITING_FOR_RPLY = 3,
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
#define P2_TO_THRESH                    150 // <- change this
#define CLS_TO_THRESH                   500
#define CCA_EN                          0
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
int sniff_cnt = 0;
int wac1_sniffs = 0;
int tot_wac1_scan = 0;
clock_time_t sniff1_start_time, WaC1_detection_time, to_tim; 
int wac1_sniff_inteval = SNIFF_INTERVAL;
int to_counter = 0;
uint32_t tx_timestamp, reply_sniff_timestamp;
/*---------------------------------------------------------------------------*/

void tx_ok_cb(const dwt_cb_data_t *cb_data){
  tx_timestamp = dwt_readtxtimestamphi32();
  printf("TX: %d, %d, %d\n", node_id, 0, tot_sniffs); 
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
  
  case WAITING_FOR_RPLY:
    if (rx_payload[0] == 0xef){
      printf(" ------------- REPLY -------------\n");
    }else{
      printf(" ------------- FALSE-REP -------------\n");
    }
      
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
    detection_status = RX_WAK_P2;
    WaC1_detection_time = clock_time();
    printf("Detected WaC1: %d, %d, %d, %d\n", node_id, wac1_sniffs, tot_wac1_scan, WaC1_detection_time - sniff1_start_time);
    wac1_sniffs = 0;
    break;
  case RX_WAK_P2:
#if (TS_MODE)
    detection_status = RX_WAK_P3;
#else
    detection_status = WAITING;
#endif
    printf("Detected WaC2: %d\n", node_id);
    break;
  
  case CCA:
    detection_status = PKT_DETECTED;
    break;
  
  case WAITING_FOR_RPLY:
    printf("bad backet ERR\n");
    break;
 
  default:
    break;
  }
}


void rx_to_cb(const dwt_cb_data_t *cb_data){
  dwt_forcetrxoff();
  to_counter++;
  switch (detection_status){
  case RX_WAK_P2:
    detection_status = RX_WAK_P2;
    break;
  case CCA:
    detection_status = RDY_TO_TX;
  break;
  default:
    detection_status = RX_WAK_P1;
    
    break;
  }
}
/*---------------------------------------------------------------------------*/


PROCESS_THREAD(report_stat, ev, data){
  static struct etimer et;
  PROCESS_BEGIN();
  while (1){
    etimer_set(&et, CLOCK_SECOND);
    PROCESS_WAIT_UNTIL(etimer_expired(&et));
    printf("NODE STAT: TOT sniff cnt: %d, WaC1 sniffs: %d, 0x%x\n", tot_sniffs, wac1_sniffs, to_counter);
  }
  PROCESS_END();
}




void dwt_init(){
  dw1000_arch_init();
  dw1000_reset_cfg();

dw1000_set_isr(dwt_isr);
  /* Register TX/RX callbacks. */
  dwt_setcallbacks(&tx_ok_cb, &rx_ok_cb, &rx_to_cb, &rx_err_cb);
  /* Enable wanted interrupts (TX confirmation, RX good frames, RX timeouts and RX errors). */
  dwt_setinterrupt(DWT_INT_TFRS | DWT_INT_RFCG | DWT_INT_RFTO | DWT_INT_RXPTO |
                   DWT_INT_RPHE | DWT_INT_RFCE | DWT_INT_RFSL | DWT_INT_SFDT |
                   DWT_INT_ARFE, 1);

  
}


PROCESS_THREAD(range_process, ev, data)
{
  static struct etimer et;
  static struct etimer timeout;
  static int status;
  
  
  
  PROCESS_BEGIN();


  dwt_init();
  
  if(deployment_set_node_id_ieee_addr()){
    printf("NODE addr set successfully: %d, CCA_EN: %d\n", node_id, CCA_EN);
  }else{
    printf("Failed to set nodeID\n");
  }

  printf("STARTING reliability EXP2: SNIFF_INTERVAL %d, RAPID_SNIFF_INT %d, WAC_TO %d, %d\n", SNIFF_INTERVAL, RAPID_SNIFF_INTERVAL, P2_TO_THRESH, PDTO);
  dwt_configure(&config);
  dwt_configuretxrf(&txConf);
  PROCESS_WAIT_UNTIL(etimer_expired(&et));
  dwt_forcetrxoff();
  dwt_setpreambledetecttimeout(PDTO);  
  payload[2] = node_id;
  clock_init();
  memcpy(&payload[2], (uint16_t *) &node_id, 2);
  detection_status = RX_WAK_P1;
  

  while (1){
    etimer_set(&et, 1);
    PROCESS_WAIT_UNTIL(etimer_expired(&et));
    if (detection_status == RX_WAK_P1){
      dwt_init();
      config_change = true;
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
      

      
      etimer_set(&et, wac1_sniff_inteval - 11);
      PROCESS_WAIT_UNTIL(etimer_expired(&et));
      sniff1_start_time = clock_time();
      dwt_configure(&config);
      dwt_forcetrxoff();
      dwt_rxreset();
      if(dwt_rxenable(DWT_START_RX_IMMEDIATE) != DWT_SUCCESS){
        printf("gholi\n");
      }
      etimer_set(&et, 10);
      PROCESS_WAIT_UNTIL(etimer_expired(&et));
      wac1_sniff_inteval = SNIFF_INTERVAL;
      P2_timeout = 0;
      CLS_timeout = 0;
      back_off = 0;
      sniff_cnt = 1;
      tot_sniffs++;
      wac1_sniffs++;
      tot_wac1_scan++;
    }
    if (detection_status == RX_WAK_P2){
      config_change = false;
      if (config.rxCode != WAC2_PC){
        config_change = true;
        config.rxCode = WAC2_PC;
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
      
      dwt_forcetrxoff();
      dwt_rxreset();
      etimer_set(&et, RAPID_SNIFF_INTERVAL - 11);
      PROCESS_WAIT_UNTIL(etimer_expired(&et));
      dwt_rxenable(DWT_START_RX_IMMEDIATE);
      P2_timeout += RAPID_SNIFF_INTERVAL;
      etimer_set(&et, 10);
      PROCESS_WAIT_UNTIL(etimer_expired(&et));
      if (detection_status == RX_WAK_P2 &&  P2_timeout > P2_TO_THRESH){
        dwt_forcetrxoff();
        dwt_rxreset();
        detection_status = RX_WAK_P1;
        to_tim = clock_time();
        printf("TO %d, %d, %d\n", sniff_cnt, tot_sniffs, to_tim - WaC1_detection_time);
        wac1_sniff_inteval = 30;
      }
      sniff_cnt += 1;
      tot_sniffs++;
    }
    if (detection_status == WAITING){
      config.prf = DWT_PRF_64M;
      config.rxCode = 9;
      config.rxPAC = DWT_PAC8;
      config.sfdTO = SFD_TO;
      dwt_configure(&config);
      
      
      etimer_set(&et, RAPID_SNIFF_INTERVAL + 20);
      PROCESS_WAIT_UNTIL(etimer_expired(&et));
#if (CCA_EN == 1)
      detection_status = CCA;
#else
      detection_status = RDY_TO_TX;
#endif
    }
    if (detection_status == CCA){
      dwt_setpreambledetecttimeout(200);
      dwt_rxenable(DWT_START_RX_IMMEDIATE);
      unsigned short cca_wait = 2 + (random_rand() % 5);
      etimer_set(&et, cca_wait);
      PROCESS_WAIT_UNTIL(etimer_expired(&et));
    }

    if (detection_status == PKT_DETECTED){
      detection_status = CCA;
      back_off++;
      dwt_rxenable(DWT_START_RX_IMMEDIATE);
    }
    if (detection_status == RDY_TO_TX){
      config.rxPAC = PAC;
      config.prf = DWT_PRF_64M;
      config.rxCode = 9;
      config.sfdTO = 8000;
      dwt_configure(&config);
      dwt_forcetrxoff();
      payload[0] = 0xad;
      memcpy(&payload[2], (uint16_t *) &node_id, 2);
      dwt_writetxdata(sizeof(payload), payload, 0);
      dwt_writetxfctrl(sizeof(payload), 0, 0);
      
      if(dwt_starttx(DWT_START_TX_IMMEDIATE) != DWT_SUCCESS){
        printf("TX ERR\n");

        printf("oh no\n");
      }

      tot_sniffs ++;
      

      
      etimer_set(&et,6);
      PROCESS_WAIT_UNTIL(etimer_expired(&et));

      
      // reply_sniff_timestamp = tx_timestamp + (5000 * UUS_TO_DWT_TIME) >> 8;
      detection_status = WAITING_FOR_RPLY;
      dwt_forcetrxoff();
      dwt_rxreset();
      dwt_setpreambledetecttimeout(PDTO);
      dwt_setdelayedtrxtime(reply_sniff_timestamp);
      printf("reply result %d\n", dwt_rxenable(DWT_START_RX_IMMEDIATE) == DWT_SUCCESS);
      
      etimer_set(&et, 100);
      PROCESS_WAIT_UNTIL(etimer_expired(&et));
      
      detection_status = RX_WAK_P1;
      wac1_sniff_inteval = SNIFF_INTERVAL;
      dwt_forcetrxoff();
      dwt_setpreambledetecttimeout(PDTO);
      etimer_set(&et, 10);
      PROCESS_WAIT_UNTIL(etimer_expired(&et));
    }

  }
  
  PROCESS_END();
}
/*---------------------------------------------------------------------------*/