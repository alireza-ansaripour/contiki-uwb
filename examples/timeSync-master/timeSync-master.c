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
AUTOSTART_PROCESSES(&ts_process);
/*---------------------------------------------------------------------------*/

typedef struct {
  uint8_t packet_type;
  uint8_t src;
  uint8_t dst;
  uint32_t seq;
  uint8_t payload[10];
  uint16_t crc;
} packet_format_t;

typedef struct{
  uint8_t sender_id;
  uint8_t config_id;
} time_sync_payload;


void instance_config(){
  printf("instance ID: 0x%ld\n", *STM32_UUID);


}


#define MAX_BUF_LEN 36
static uint8_t rtx_buf[MAX_BUF_LEN];
int rx_cnt = 0;
int rx_err_cnt = 0;
int tx_num = 0;

void rx_ok_cb(const dwt_cb_data_t *cb_data){
  rx_cnt += 1;
  // printf("RX Ok CB receiver: %d\n", rx_cnt);
  dwt_forcetrxoff();
  dwt_rxenable(DWT_START_RX_IMMEDIATE);
  
}

void rx_err_cb(const dwt_cb_data_t *cb_data){
  rx_err_cnt += 1;
  dwt_forcetrxoff();
  dwt_rxenable(DWT_START_RX_IMMEDIATE);
  printf("RX ERR CB receiver: %ld\n", cb_data->status);
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


uint8_t senders[] = {50,51,52,53,56,58,63,64,71,72,73,74,75,76};
// uint8_t senders[] = {121,122,123,125,126,127,128,129,130,132,133,135,136,138,139,140};

int turn = 0, configID = 0;
packet_format_t pkt;
PROCESS_THREAD(ts_process, ev, data)
{
  static struct etimer et;
  
  PROCESS_BEGIN();
  printf("TimeSync Node\n");
  dwt_setcallbacks(NULL, &rx_ok_cb, NULL, &rx_err_cb);
  instance_config();
  
  pkt.src = 0x0;
  pkt.packet_type = 0x01;
  pkt.seq = 0x0;

  lcd_display_str("Time sync");
  while(1){
    time_sync_payload *ts = (time_sync_payload *) pkt.payload;
    ts->sender_id = senders[turn];
    turn = (turn + 1);
    ts->config_id = configID;
    if (turn == sizeof(senders)){
      turn = 0;
      configID = configID + 1;
    }
    dwt_forcetrxoff();
    printf("TX MASTER: TS sent %ld, %ld\n", pkt.seq, pkt.payload[0]);
    dwt_writetxdata(sizeof(pkt), (uint8_t *) &pkt, 0);
    dwt_writetxfctrl(sizeof(pkt), 0, 0);

    dwt_starttx(DWT_START_TX_IMMEDIATE);
    etimer_set(&et, CLOCK_SECOND * 10);
    
    PROCESS_WAIT_UNTIL(etimer_expired(&et));
    pkt.seq += 1;

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
