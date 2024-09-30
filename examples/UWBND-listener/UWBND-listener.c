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
// #include "nrf.h"
#include <sys/node-id.h>
#include "core/net/linkaddr.h"


/*---------------------------------------------------------------------------*/
PROCESS(range_process, "Test range process");
PROCESS(report_stat, "Test range process");
AUTOSTART_PROCESSES(&range_process);
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

#define IPI                             100
#define SNIFF_INTERVAL                  100
#define RAPID_SNIFF_INTERVAL            100
#define TIMEOUT_MS                      500

/*---------------------------------------------------------------------------*/
dwt_config_t config = {
    5, /* Channel number. */
    DWT_PRF_64M, /* Pulse repetition frequency. */
    DWT_PLEN_256, /* Preamble length. Used in TX only. */
    DWT_PAC8, /* Preamble acquisition chunk size. Used in RX only. */
    9, /* TX preamble code. Used in TX only. */
    9, /* RX preamble code. Used in RX only. */
    0, /* 0 to use standard SFD, 1 to use non-standard SFD. */
    DWT_BR_6M8, /* Data rate. */
    DWT_PHRMODE_STD, /* PHY header mode. */
    (2) /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
};

dwt_txconfig_t txConf = {
    0xC9, //PGDelay
    0x18181818 //30 dB
};


uint8_t payload[] = {0xad, 0, 0, 0, 0, 0};
uint8_t rx_payload[20];
DETECTION_STATUS detection_status = RX_WAK_P1;
clock_time_t wac1_detection_time, current_time;
clock_time_t timeOut = TIMEOUT_MS;
int wac1_sniff_interval = SNIFF_INTERVAL;


/*---------------------------------------------------------------------------*/

PROCESS_THREAD(report_stat, ev, data){
  static struct etimer et;
  PROCESS_BEGIN();
  while (1){
    etimer_set(&et, CLOCK_SECOND);
    PROCESS_WAIT_UNTIL(etimer_expired(&et));
    
  }
  PROCESS_END();
}



uint32_t status_reg;
int phase;

void rx_ok_cb(const dwt_cb_data_t *cb_data){
  dwt_forcetrxoff();
  dwt_rxenable(DWT_START_RX_IMMEDIATE);
  // printf("TX OK Sender\n");
}

void rx_err_cb(const dwt_cb_data_t *cb_data){
  dwt_forcetrxoff();
  if (detection_status == RX_WAK_P1){
    printf("WAC1dfdfdfd detected\n");
    detection_status = RX_WAK_P2;
    wac1_detection_time = clock_time();
  }else{
    printf("WAC2 dfdfdfdetected\n");
    detection_status = WAITING;
  }
}

void rx_to_cb(const dwt_cb_data_t *cb_data){
  dwt_forcetrxoff();
}






PROCESS_THREAD(range_process, ev, data){
  static struct etimer et;
  static struct etimer timeout;
  static int status;

  
  
  PROCESS_BEGIN();


  // dwt_setcallbacks(NULL, &rx_ok_cb, &rx_to_cb, &rx_err_cb);
  if(deployment_set_node_id_ieee_addr()){
    printf("NODE addr set successfully: %d\n", node_id);
  }else{
    printf("Failed to set nodeID\n");
  }


  printf("STARTING reliability EXP2\n");
  
  PROCESS_WAIT_UNTIL(etimer_expired(&et));
  dwt_forcetrxoff();
  dwt_setpreambledetecttimeout(3);  
  payload[2] = node_id;
  clock_init();
  memcpy(&payload[2], (uint16_t *) &node_id, 2);

  // dw1000_set_isr(dwt_isr);
  // dwt_setinterrupt(DWT_INT_TFRS | DWT_INT_RFCG | DWT_INT_RFTO | DWT_INT_RXPTO |
  //                 DWT_INT_RPHE | DWT_INT_RFCE | DWT_INT_RFSL | DWT_INT_SFDT |
  //                 DWT_INT_ARFE, 1);
  

  switch (node_id)
  {
  case 129:
  case 74:
    config.rxCode = 3;
    config.prf = DWT_PRF_16M;
    break;

  case 130:
  case 71:
    config.rxCode = 9;
    config.prf = DWT_PRF_64M;
    break;
  
  default:
    break;
  }
  
  dwt_configure(&config);
  dwt_configuretxrf(&txConf);
  
  wac1_sniff_interval = SNIFF_INTERVAL;
  etimer_set(&et, CLOCK_SECOND);
  PROCESS_WAIT_UNTIL(etimer_expired(&et));

  while (1){
    
    dwt_forcetrxoff();
    etimer_set(&et, 2);
    PROCESS_WAIT_UNTIL(etimer_expired(&et));
    dwt_forcetrxoff();
    dwt_rxreset();
    printf("Sniffing\n");
    dwt_rxenable(DWT_START_RX_IMMEDIATE);
    etimer_set(&et, 1);
    PROCESS_WAIT_UNTIL(etimer_expired(&et));
    status_reg = dwt_read32bitreg(SYS_STATUS_ID);
    if (status_reg & SYS_STATUS_ALL_RX_ERR){
      wac1_detection_time = clock_time();
      printf("WAC1 detected\n");
      
      dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);
    }else{
      dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_TO);
    }
    wac1_sniff_interval = SNIFF_INTERVAL;
    detection_status = RX_WAK_P2;

  }
  PROCESS_END();
}
/*---------------------------------------------------------------------------*/


    // status_reg = dwt_read32bitreg(SYS_STATUS_ID);
    
    // if (status_reg & SYS_STATUS_RXFCG){
    //   dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG);
    //   printf("RXOK\n");
    // }
    // if (status_reg & SYS_STATUS_ALL_RX_TO){
    //   // printf("RX TO\n");
    //   dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);
    // }
    // if (status_reg & SYS_STATUS_ALL_RX_ERR){

    //   dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);
    //   phase = (phase + 1) % 2;

      

    //   if (status_reg & SYS_STATUS_RXPHE){printf("PHE ERR\n");}
    //   if (status_reg & SYS_STATUS_RXFCE){printf("FCE ERR\n");}
    //   if (status_reg & SYS_STATUS_RXSFDTO){printf("SFDTO ERR\n");}
    //   if (status_reg & SYS_STATUS_RXRFSL){printf("FSL ERR\n");}
    // }