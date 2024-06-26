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
/*---------------------------------------------------------------------------*/
PROCESS(range_process, "Test range process");
PROCESS(cnt_rx_frames, "count rx frames");
AUTOSTART_PROCESSES(&range_process, &cnt_rx_frames);
/*---------------------------------------------------------------------------*/






uint8_t msg[] = {0, 0, 0, 0, 0 , 0, 0 ,0};
static const frame802154_t default_header = {
  .fcf = {
    .frame_type        = 1, // data
    .security_enabled  = 0,
    .frame_pending     = 0,
    .ack_required      = 0,
    .panid_compression = 1, // suppress src PAN ID
    .sequence_number_suppression = 0,
    .ie_list_present   = 0,
    .dest_addr_mode    = (LINKADDR_SIZE == 8) ? 3 : 2,
    .frame_version     = 0,
    .src_addr_mode     = (LINKADDR_SIZE == 8) ? 3 : 2,
  },
  .dest_pid = IEEE802154_PANID,
  .src_pid  = IEEE802154_PANID,
};


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
  printf("RX ERR CB receiver: %d\n", cb_data->status);
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


PROCESS_THREAD(range_process, ev, data)
{
  static struct etimer et;
  // static struct etimer timeout;
  // static int status;
  uint8_t irq_status;

  PROCESS_BEGIN();
  frame802154_t frame2 = default_header;
  printf("TEST receiver 7\n");
  dwt_setcallbacks(NULL, &rx_ok_cb, NULL, &rx_err_cb);
  dwt_forcetrxoff();
  dwt_rxenable(DWT_START_RX_IMMEDIATE);
  while (1){
    etimer_set(&et, CLOCK_SECOND);
    PROCESS_WAIT_UNTIL(etimer_expired(&et));
  }
  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
