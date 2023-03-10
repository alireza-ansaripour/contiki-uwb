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

/*---------------------------------------------------------------------------*/
PROCESS(range_process, "Test range process");
PROCESS(count_tx_packets, "Counts transmitted packets");
PROCESS(start_tx, "start sending");

AUTOSTART_PROCESSES(&range_process);

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

typedef struct{
  uint8_t sender_id;
  uint32_t ts_seq;
} data_payload;

packet_format_t pkt;
packet_format_t txpkt;
int tx_num = 0;
uint8_t nodeID;


dwt_config_t ts_config =   {
    3, /* Channel number. */
    DWT_PRF_64M, /* Pulse repetition frequency. */
    DWT_PLEN_1024, /* Preamble length. Used in TX only. */
    DWT_PAC32, /* Preamble acquisition chunk size. Used in RX only. */
    9, /* TX preamble code. Used in TX only. */
    9, /* RX preamble code. Used in RX only. */
    0, /* 0 to use standard SFD, 1 to use non-standard SFD. */
    DWT_BR_6M8, /* Data rate. */
    DWT_PHRMODE_STD, /* PHY header mode. */
    (8000 + 1 + 64 - 64) /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
};


dwt_config_t config_array[] = {
    {
        3, /* Channel number. */
        DWT_PRF_64M, /* Pulse repetition frequency. */
        DWT_PLEN_1024, /* Preamble length. Used in TX only. */
        DWT_PAC8, /* Preamble acquisition chunk size. Used in RX only. */
        9, /* TX preamble code. Used in TX only. */
        9, /* RX preamble code. Used in RX only. */
        0, /* 0 to use standard SFD, 1 to use non-standard SFD. */
        DWT_BR_850K, /* Data rate. */
        DWT_PHRMODE_STD, /* PHY header mode. */
        (8000 + 1 + 64 - 64) /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
    },
    {
        3, /* Channel number. */
        DWT_PRF_64M, /* Pulse repetition frequency. */
        DWT_PLEN_1024, /* Preamble length. Used in TX only. */
        DWT_PAC16, /* Preamble acquisition chunk size. Used in RX only. */
        9, /* TX preamble code. Used in TX only. */
        9, /* RX preamble code. Used in RX only. */
        0, /* 0 to use standard SFD, 1 to use non-standard SFD. */
        DWT_BR_850K, /* Data rate. */
        DWT_PHRMODE_STD, /* PHY header mode. */
        (8000 + 1 + 64 - 64) /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
    },
    {
        3, /* Channel number. */
        DWT_PRF_64M, /* Pulse repetition frequency. */
        DWT_PLEN_1024, /* Preamble length. Used in TX only. */
        DWT_PAC32, /* Preamble acquisition chunk size. Used in RX only. */
        9, /* TX preamble code. Used in TX only. */
        9, /* RX preamble code. Used in RX only. */
        0, /* 0 to use standard SFD, 1 to use non-standard SFD. */
        DWT_BR_850K, /* Data rate. */
        DWT_PHRMODE_STD, /* PHY header mode. */
        (8000 + 1 + 64 - 64) /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
    },
    {
        3, /* Channel number. */
        DWT_PRF_64M, /* Pulse repetition frequency. */
        DWT_PLEN_1024, /* Preamble length. Used in TX only. */
        DWT_PAC64, /* Preamble acquisition chunk size. Used in RX only. */
        9, /* TX preamble code. Used in TX only. */
        9, /* RX preamble code. Used in RX only. */
        0, /* 0 to use standard SFD, 1 to use non-standard SFD. */
        DWT_BR_850K, /* Data rate. */
        DWT_PHRMODE_STD, /* PHY header mode. */
        (8000 + 1 + 64 - 64) /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
    },
};

dwt_txconfig_t txConf_array[] = {
    {
        0xC9, //PGDelay
        0x18181818 //30 dB
    },
    {
        0xC9, //PGDelay
        0x12121212 //27
    },
    {
        0xC9, //PGDelay
        0x0C0C0C0C //24
    },
    {
        0xC9, //PGDelay
        0x07070707 //21
    },
    {
        0xC9, //PGDelay
        0x01010101 //18
    },
    {
        0xC9, //PGDelay
        0x20202020 //15
    },
    {
        0xC9, //PGDelay
        0x40404040 //12
    },
    {
        0xC9, //PGDelay
        0x60606060 //9
    },
    {
        0xC9, //PGDelay
        0x80808080 //6
    },
    {
        0xC9, //PGDelay
        0xA0A0A0A0 //3
    },
    {
        0xC9, //PGDelay
        0xC0C0C0C0 //0
    }
};



void set_node_id(){
  switch (STM32_UUID[2])
  {
    case 0x43133243: 
      nodeID = 52;
      break;
    case 0x43184141: 
      nodeID = 51;
      break;
    case 0x43133244: 
      nodeID = 50;
      break;
    case 0x43115436: 
      nodeID = 64;
      break;
    case 0x43174444: 
      nodeID = 53;
      break;
    case 0x43184234: 
      nodeID = 63;
      break;
    case 0x43153731: 
      nodeID = 56;
      break;
    case 0x43074935: 
      nodeID = 75;
      break;
    case 0x43012947: 
      nodeID = 71;
      break;
    case 0x43103042: 
      nodeID = 74;
      break;
    case 0x43133349: 
      nodeID = 73;
      break;
    case 0x43124426: 
      nodeID = 72;
      break;
    case 0x43075461: 
      nodeID = 76;
      break;
    case 0x43054345:
      nodeID = 58;
      break;
    case 0x43056438: 
      nodeID = 138;
      break;
    case 0x43063624: 
      nodeID = 139;
      break;
    case 0x43054361: 
      nodeID = 123;
      break;
    case 0x43014626: 
      nodeID = 137;
      break;
    case 0x43053146: 
      nodeID = 127;
      break;
    case 0x43044838: 
      nodeID = 132;
      break;
    case 0x43014629: 
      nodeID = 136;
      break;
    case 0x43044736: 
      nodeID = 130;
      break;
    case 0x43133861: 
      nodeID = 129;
      break;
    case 0x43053060: 
      nodeID = 133;
      break;
    case 0x43044234: 
      nodeID = 134;
      break;
    case 0x43054366: 
      nodeID = 124;
      break;
    case 0x43013138: 
      nodeID = 140;
      break;
    case 0x43053057: 
      nodeID = 135;
      break;
    case 0x43254525: 
      nodeID = 121;
      break;
    case 0x43214744: 
      nodeID = 128;
      break;
    case 0x43014358: 
      nodeID = 122;
      break;
    case 0x43014547: 
      nodeID = 126;
      break;
    case 0x43044136: 
      nodeID = 131;
      break;
    case 0x43013143: 
      nodeID = 125;
      break;
      
    default:
      printf("Device NodeID: 0x%x\n", STM32_UUID[2]);
  }

}


void tx_ok_cb(const dwt_cb_data_t *cb_data){
  tx_num ++;
  // printf("TX OK Sender\n");
}

void rx_err_cb(const dwt_cb_data_t *cb_data){
  
  dwt_forcetrxoff();
  dwt_rxenable(DWT_START_RX_IMMEDIATE);
  printf("RX ERR: %x\n", cb_data->status);
  // printf("TX OK Sender\n");
}


int looking = 0;
dwt_rxdiag_t diag_info;

PROCESS(change_ts_config, "changes to ts config");
PROCESS_THREAD(change_ts_config, env, data){
  static struct etimer et;
  PROCESS_BEGIN();
  etimer_set(&et, CLOCK_SECOND * 9);
  PROCESS_WAIT_UNTIL(etimer_expired(&et));
  dwt_forcetrxoff();
  dwt_configure(&ts_config);
  dwt_rxenable(DWT_START_RX_IMMEDIATE);
  printf("INFO: Switching to TS conf\n");
  PROCESS_END();
}

PROCESS(handle_rx, "handles incoming messages");
PROCESS_THREAD(handle_rx, env, data){
  static struct etimer et;
  PROCESS_BEGIN();
  if (pkt.packet_type == 1 || pkt.packet_type == 2){
    time_sync_payload * payload = (time_sync_payload *)pkt.payload;
    if (pkt.seq >= looking){
      looking = pkt.seq + 1;
      
      uint8_t confIndex = payload->config_id % 4;
      uint8_t pwrIndex = 7;
      // uint8_t confIndex = 5;
      // uint8_t pwrIndex = payload->config_id % 11;
      printf("TS  : %ld, %d, %ld\n",payload->sender_id, confIndex, pkt.seq);
      printf("CONF: {chan:%d, PC:%d, pac:%d, DR:%d},",config_array[confIndex].chan,config_array[confIndex].prf, config_array[confIndex].rxPAC, config_array[confIndex].dataRate);
      printf("{TXP:0x%x}\n", txConf_array[pwrIndex].power);
      process_start(&change_ts_config, NULL);
      etimer_set(&et, CLOCK_SECOND / 2);
      PROCESS_WAIT_UNTIL(etimer_expired(&et));
      dwt_forcetrxoff();
      dwt_configure(&config_array[confIndex]);
      dwt_configuretxrf(&txConf_array[pwrIndex]);
      
      if(nodeID == payload->sender_id){
        txpkt.src = nodeID;
        txpkt.packet_type = 0x03;
        data_payload *payload = (data_payload *)txpkt.payload;
        payload->ts_seq = pkt.seq;
        printf("Start TX\n");
        process_start(&start_tx, (void *)&pkt.seq);

      }else{

        dwt_rxreset();
        dwt_rxenable(DWT_START_RX_IMMEDIATE);
      }
        
    }
  }
  if(pkt.packet_type == 0x03){
    
    dwt_readdiagnostics(&diag_info);
    printf("DATA:%d, %ld, %d, %d, %d, %d, %d, %d, %d, %d\n", 
    pkt.src, 
    pkt.seq, 
    diag_info.maxGrowthCIR, 
    diag_info.rxPreamCount, 
    diag_info.firstPathAmp1, 
    diag_info.firstPathAmp2, 
    diag_info.firstPathAmp3,
    diag_info.stdNoise,
    diag_info.maxNoise,
    diag_info.pacNonsat
    );
    
  }
  PROCESS_END();
}


void rx_ok_cb(const dwt_cb_data_t *cb_data){
  dwt_readrxdata((uint8_t *)&pkt, cb_data->datalength,0);
  dwt_forcetrxoff();
  dwt_rxenable(DWT_START_RX_IMMEDIATE);
  process_start(&handle_rx, NULL);
  
}


int cnter;
PROCESS_THREAD(start_tx, ev, data){
  static struct etimer et;
  PROCESS_BEGIN();
  etimer_set(&et, CLOCK_SECOND);
  dwt_forcetrxoff();
  PROCESS_WAIT_UNTIL(etimer_expired(&et));
  for (cnter =0 ; cnter< 100; cnter++){
    dwt_writetxdata(sizeof(txpkt), (uint8_t *)&txpkt, 0);
    dwt_writetxfctrl(sizeof(txpkt),0,0);
    
    int res = dwt_starttx(DWT_START_TX_IMMEDIATE);
    if (res != DWT_SUCCESS)
      printf("TX FAILED\n");

    txpkt.seq++;
    etimer_set(&et, CLOCK_SECOND/40);
    PROCESS_WAIT_UNTIL(etimer_expired(&et));
  }
  PROCESS_END();
}

PROCESS_THREAD(count_tx_packets, ev, data){

  static struct etimer et;

  PROCESS_BEGIN();

  etimer_set(&et, CLOCK_SECOND * 100);
  PROCESS_WAIT_UNTIL(etimer_expired(&et));
  printf("TX report: %d\n", tx_num);
  PROCESS_END();
}


PROCESS_THREAD(range_process, ev, data)
{
  static struct etimer et;
  // static struct etimer timeout;
  // static int status;
  uint8_t irq_status;

  PROCESS_BEGIN();
  set_node_id();

  dwt_setcallbacks(&tx_ok_cb, &rx_ok_cb, NULL, &rx_err_cb);

  printf("TEST sender 2\n");
  dwt_forcetrxoff();
  dwt_rxenable(DWT_START_RX_IMMEDIATE);
  while (1){
    // dwt_forcetrxoff();
    etimer_set(&et, CLOCK_SECOND * 2);
    PROCESS_WAIT_UNTIL(etimer_expired(&et));
  }
  printf("shouldn't be here\n");

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
