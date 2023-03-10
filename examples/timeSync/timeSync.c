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

#include "core/net/linkaddr.h"

#define STM32_UUID ((uint32_t *)0x1ffff7e8)
#define ROLE_TS_MASTER 0x0
#define ROLE_TS_SLAVE  0x1

/*---------------------------------------------------------------------------*/
PROCESS(ts_process, "Test range process");
PROCESS(cnt_rx_frames, "count rx frames");
PROCESS(tx_ts, "send ts frame");
AUTOSTART_PROCESSES(&ts_process);
/*---------------------------------------------------------------------------*/


void instance_config(){
  printf("instance ID: 0x%ld\n", *STM32_UUID);


}

typedef struct{
  uint8_t packet_type;
  uint8_t src;
  uint8_t dst;
  uint32_t seq;
  uint8_t payload[10];
  uint16_t crc;
}packet_format_t;


#define MAX_BUF_LEN 70
static uint8_t rtx_buf[MAX_BUF_LEN];
int rx_cnt = 0;
int rx_err_cnt = 0;
int tx_num = 0;
packet_format_t pkt;

PROCESS_THREAD(tx_ts, env, data){
  static struct etimer et;
  PROCESS_BEGIN();
  dwt_forcetrxoff();
  dwt_writetxdata(sizeof(pkt), (uint8_t *) &pkt, 0);
  dwt_writetxfctrl(sizeof(pkt), 0, 0);
  uint32_t wait = (STM32_UUID[2] % 20);

  // etimer_set(&et, wait);
  // PROCESS_WAIT_UNTIL(etimer_expired(&et));
  int ret = dwt_starttx(DWT_START_TX_IMMEDIATE);
  if (ret != DWT_SUCCESS)
    printf("TX failed\n");
  printf("RX Ok CB receiver: %d:%d, %d\n", pkt.packet_type, pkt.src, pkt.seq);
  PROCESS_END();
}




uint32_t looking_seq = 0;
void rx_ok_cb(const dwt_cb_data_t *cb_data){
  
  rx_cnt += 1;
  dwt_readrxdata((uint8_t *) &pkt, cb_data->datalength, 0);
  if (pkt.packet_type == 0x01 || pkt.packet_type == 0x02){
    if (pkt.seq >= looking_seq ){
      looking_seq = pkt.seq + 1;
      pkt.src = 0x2;
      pkt.packet_type = 0x02;
      lcd_display_str("Got ts packet");
      process_start(&tx_ts, NULL);
      
    }else{
      dwt_forcetrxoff();
      dwt_rxenable(DWT_START_RX_IMMEDIATE);
    }
      
  }else{
    dwt_forcetrxoff();
    dwt_rxenable(DWT_START_RX_IMMEDIATE);
  }
  
}

void rx_err_cb(const dwt_cb_data_t *cb_data){
  rx_err_cnt += 1;
  dwt_forcetrxoff();
  dwt_rxenable(DWT_START_RX_IMMEDIATE);
  printf("RX ERR CB receiver: %d\n", cb_data->status);
}

void tx_ok_cb(const dwt_cb_data_t *cb_data){
  
  dwt_forcetrxoff();
  dwt_rxenable(DWT_START_RX_IMMEDIATE);
  
}

PROCESS_THREAD(cnt_rx_frames, ev, data){
  static struct etimer et;

  PROCESS_BEGIN();
  printf("STARTING RX task\n");
  etimer_set(&et, CLOCK_SECOND * 100);
  PROCESS_WAIT_UNTIL(etimer_expired(&et));
  printf("RX report: %d, %d\n", rx_cnt, rx_err_cnt);
  PROCESS_END();
}


PROCESS_THREAD(ts_process, ev, data)
{
  static struct etimer et;

  PROCESS_BEGIN();
  dwt_setcallbacks(&tx_ok_cb, &rx_ok_cb, NULL, &rx_err_cb);
  lcd_display_str("ts node");
  dwt_forcetrxoff();
  dwt_rxenable(DWT_START_RX_IMMEDIATE);
  while(1){

  }

  // dwt_forcetrxoff();
  // dwt_rxenable(DWT_START_RX_IMMEDIATE);
  // while (1){
  //   etimer_set(&et, CLOCK_SECOND);
  //   PROCESS_WAIT_UNTIL(etimer_expired(&et));
  // }
  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
