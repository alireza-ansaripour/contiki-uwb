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
PROCESS(cnt_sniff_counter, "Test range process");
AUTOSTART_PROCESSES(&range_process, &cnt_sniff_counter);

#define UUS_TO_DWT_TIME     65536

dwt_config_t config = {
    1, /* Channel number. */
    DWT_PRF_64M, /* Pulse repetition frequency. */
    DWT_PLEN_4096, /* Preamble length. Used in TX only. */
    DWT_PAC8, /* Preamble acquisition chunk size. Used in RX only. */
    9, /* TX preamble code. Used in TX only. */
    9, /* RX preamble code. Used in RX only. */
    0, /* 0 to use standard SFD, 1 to use non-standard SFD. */
    DWT_BR_6M8, /* Data rate. */
    DWT_PHRMODE_STD, /* PHY header mode. */
    8000 /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
};

dwt_config_t config2 = {
    1, /* Channel number. */
    DWT_PRF_16M, /* Pulse repetition frequency. */
    DWT_PLEN_4096, /* Preamble length. Used in TX only. */
    DWT_PAC8, /* Preamble acquisition chunk size. Used in RX only. */
    1, /* TX preamble code. Used in TX only. */
    1, /* RX preamble code. Used in RX only. */
    0, /* 0 to use standard SFD, 1 to use non-standard SFD. */
    DWT_BR_6M8, /* Data rate. */
    DWT_PHRMODE_STD, /* PHY header mode. */
    8000 /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
};



dwt_txconfig_t txConf = {
    0xC9, //PGDelay
    0x18181818 //30 dB
};


uint8_t payload[] = {0xad, 0, 0, 0, 0, 0};
uint8_t rx_payload[20];
int tx_counter = 0;
status = 1;

/*---------------------------------------------------------------------------*/

void tx_ok_cb(const dwt_cb_data_t *cb_data){
  // if (config.prf = DWT_PRF_64M){
  //   config.prf = DWT_PRF_16M;
  //   config.txCode = 1;
  //   config.rxCode = 1;
  // }else{
  //   config.prf = DWT_PRF_64M;
  //   config.txCode = 9;
  //   config.rxCode = 9;
  // }
  // dwt_forcetrxoff();
  // dwt_configure(&config);
  dwt_writetxfctrl(sizeof(payload), 0, 0);
  dwt_starttx(DWT_START_TX_IMMEDIATE);
  tx_counter++;
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




PROCESS_THREAD(cnt_sniff_counter, ev, data){
  static struct etimer et;
  PROCESS_BEGIN();
  while(1){
    etimer_set(&et, CLOCK_SECOND);
    PROCESS_WAIT_UNTIL(etimer_expired(&et));
    printf("---------------------- TX counter: %d ---------------------\n", tx_counter);
  }
  PROCESS_END();
}




PROCESS_THREAD(range_process, ev, data)
{
  static struct etimer et;
  static struct etimer timeout;
  static int status;
  
  

  PROCESS_BEGIN();
  static struct etimer et;
  dwt_setcallbacks(&tx_ok_cb, &rx_ok_cb, &rx_to_cb, &rx_err_cb);
  // deployment_print_id_info();
  dwt_configure(&config);
  dwt_configuretxrf(&txConf);
  dwt_forcetrxoff();
  payload[2] = 0;
  clock_init();

  dwt_writetxdata(sizeof(payload), payload, 0);
  dwt_writetxfctrl(sizeof(payload), 0, 0);
  dwt_starttx(DWT_START_TX_IMMEDIATE);

  while (1){
    etimer_set(&et, 1);
    PROCESS_WAIT_UNTIL(etimer_expired(&et));

      

  }
  
  PROCESS_END();
}
/*---------------------------------------------------------------------------*/