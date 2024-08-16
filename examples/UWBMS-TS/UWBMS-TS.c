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
#include <sys/node-id.h>
#include "net/netstack.h"

#define STM32_UUID ((uint32_t *)0x1ffff7e8)

/*---------------------------------------------------------------------------*/
PROCESS(range_process, "Test range process");
AUTOSTART_PROCESSES(&range_process);
#define FRAME_SIZE          100
/*---------------------------------------------------------------------------*/
typedef struct {
  uint8_t packet_type;
  uint16_t src;
  uint16_t dst;
  uint32_t seq;
  uint8_t payload[1200];
} packet_t;

typedef struct{
  uint8_t sender_id;
  uint8_t config_id;
} time_sync_payload;

typedef struct{
  uint8_t sender_id;
  uint32_t ts_seq;
} data_payload;


/*---------------------------------------------------------------------------*/

packet_t rxpkt;
packet_t txpkt;


dwt_config_t config =   {
    5, /* Channel number. */
    DWT_PRF_64M, /* Pulse repetition frequency. */
    DWT_PLEN_256, /* Preamble length. Used in TX only. */
    DWT_PAC32, /* Preamble acquisition chunk size. Used in RX only. */
    9, /* TX preamble code. Used in TX only. */
    9, /* RX preamble code. Used in RX only. */
    1, /* 0 to use standard SFD, 1 to use non-standard SFD. */
    DWT_BR_6M8, /* Data rate. */
    DWT_PHRMODE_EXT, /* PHY header mode. */
    (8000 + 1 + 64 - 64) /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
};

dwt_txconfig_t txConf = {
    0xC9, //PGDelay
    0x18181818 //30 dB
};

/*---------------------------------------------------------------------------*/


void tx_ok_cb(const dwt_cb_data_t *cb_data){
  txpkt.seq++;
  printf("TX OK Sender\n");
}

void rx_err_cb(const dwt_cb_data_t *cb_data){
  
  dwt_forcetrxoff();
  dwt_rxenable(DWT_START_RX_IMMEDIATE);
  printf("RX ERR: %x\n", cb_data->status);
  
}

PROCESS_THREAD(range_process, ev, data)
{
  static struct etimer et;
  uint8_t irq_status;

  PROCESS_BEGIN();
  
  dwt_setcallbacks(&tx_ok_cb, NULL, NULL, &rx_err_cb);

  dwt_configure(&config);
  dwt_configuretxrf(&txConf);
  dwt_forcetrxoff();

  txpkt.packet_type = 1;
  txpkt.src = 0;
  txpkt.dst = 0xffffffff;
  txpkt.seq = 0;

  dwt_writetxdata(FRAME_SIZE, (uint8_t *) &txpkt, 0);
  dwt_writetxfctrl(FRAME_SIZE, 0, 0);



  while (1){
    // dwt_forcetrxoff();
    etimer_set(&et, CLOCK_SECOND);
    PROCESS_WAIT_UNTIL(etimer_expired(&et));
    dwt_forcetrxoff();
    dwt_writetxdata(20, (uint8_t *) &txpkt, 0);
    dwt_writetxfctrl(FRAME_SIZE, 0, 0);
    dwt_starttx(DWT_START_TX_IMMEDIATE);
  }
  printf("shouldn't be here\n");

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
