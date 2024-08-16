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
#define CALLBACKS_H_
/*---------------------------------------------------------------------------*/

#define FRAME_SIZE          1022
#define PACKET_TS           1
#define PACKET_DA           2
#define UUS_TO_DWT_TIME     65536
#define TS_WAIT             0

#define UUS_TO_DWT_TIME 65536


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

typedef struct{
  uint8_t tx_PC;
  uint16_t packet_len;
  uint16_t tx_IPI_ms;
  uint16_t tx_wait_us;
  uint16_t tx_packet_num;
  uint32_t end_seq;
} sender_info_t;

/*---------------------------------------------------------------------------*/

packet_t rxpkt;
packet_t txpkt;
sender_info_t instance_info;

static struct etimer et;

dwt_config_t config =   {
    5, /* Channel number. */
    DWT_PRF_64M, /* Pulse repetition frequency. */
    DWT_PLEN_128, /* Preamble length. Used in TX only. */
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

void rx_ok_cb(const dwt_cb_data_t *cb_data){
  uint32_t rx_time = dwt_readrxtimestamphi32();
  dwt_forcetrxoff();
  dwt_readrxdata((uint8_t *) &rxpkt, cb_data->datalength, 0);
  uint32_t tx_time = rx_time + ((UUS_TO_DWT_TIME * instance_info.tx_wait_us) >> 8);
  dwt_setdelayedtrxtime(tx_time);
  dwt_writetxdata(20, (uint8_t *) &txpkt, 0);
  dwt_writetxfctrl(instance_info.packet_len, 0, 0);

  if (rxpkt.packet_type == PACKET_TS){
    instance_info.end_seq = txpkt.seq + instance_info.tx_packet_num;
    dwt_writetxfctrl(FRAME_SIZE, 0, 0);
    if (dwt_starttx(DWT_START_TX_DELAYED) == DWT_SUCCESS){
      printf("TS Frame: %d\n", rxpkt.seq);
    }else{
      printf("TX failed \n");
    }
    
  }else{
    printf("Something else\n");
  }
  
}

void tx_ok_cb(const dwt_cb_data_t *cb_data){
  txpkt.seq++;
#if TS_WAIT == 1
  if (txpkt.seq < instance_info.end_seq){
    dwt_forcetrxoff();
    dwt_writetxdata(20, (uint8_t *) &txpkt, 0);
    dwt_writetxfctrl(instance_info.packet_len, 0, 0);
    dwt_starttx(DWT_START_TX_IMMEDIATE);
  }else{
    dwt_forcetrxoff();
    dwt_rxenable(DWT_START_RX_IMMEDIATE);
  }
  printf("TX done\n");
#else
  dwt_forcetrxoff();
  dwt_writetxfctrl(instance_info.packet_len, 0, 0);
  dwt_starttx(DWT_START_TX_IMMEDIATE);
  dwt_writetxdata(20, (uint8_t *) &txpkt, 0);
  printf("TX done NS\n");
#endif
}

void rx_err_cb(const dwt_cb_data_t *cb_data){
  dwt_forcetrxoff();
  dwt_rxenable(DWT_START_RX_IMMEDIATE);
  printf("RXERR\n");
}


PROCESS(range_process, "Test range process");
AUTOSTART_PROCESSES(&range_process);



PROCESS_THREAD(range_process, ev, data)
{
  
  uint8_t irq_status;
  

  PROCESS_BEGIN();
  
  dwt_setcallbacks(&tx_ok_cb, rx_ok_cb, NULL, &rx_err_cb);

  if(deployment_set_node_id_ieee_addr()){
    printf("NODE addr set successfully: %d\n", node_id);
  }else{
    printf("Failed to set nodeID\n");
  }
  instance_info.packet_len = 100;
  instance_info.tx_IPI_ms = 0;

  printf("NODE ID is: %d\n", node_id);
  config.txPreambLength = DWT_PLEN_128;
  switch (node_id){
  case 64:
      instance_info.tx_PC = 10;
      instance_info.tx_wait_us = 1300;
      instance_info.tx_packet_num = 100;
      // config.txPreambLength = DWT_PLEN_1024;
    break;
  case 51:
      instance_info.tx_PC = 11;
      instance_info.tx_wait_us = 2200;
      instance_info.tx_packet_num = 100;
    break;
  
  default:
    break;
  }
  instance_info.packet_len = 1020;
  config.txCode = instance_info.tx_PC;
  dwt_configure(&config);
  dwt_configuretxrf(&txConf);
  dwt_forcetrxoff();

  txpkt.src = node_id;
  txpkt.dst = 0xffffffff;
  txpkt.seq = 0;
  txpkt.packet_type = PACKET_DA;


  dwt_writetxdata(instance_info.packet_len, (uint8_t *) &txpkt, 0);
  dwt_writetxfctrl(instance_info.packet_len, 0, 0);

#if TS_WAIT == 1
  dwt_rxenable(DWT_START_RX_IMMEDIATE);
#else
  dwt_starttx(DWT_START_TX_IMMEDIATE);
#endif
  
  

  while (1){
    // dwt_forcetrxoff();
    etimer_set(&et, CLOCK_SECOND * 2);
    PROCESS_WAIT_UNTIL(etimer_expired(&et));
  }
  printf("shouldn't be here\n");

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
