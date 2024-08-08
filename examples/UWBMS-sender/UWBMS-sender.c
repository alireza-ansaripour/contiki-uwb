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
#define FRAME_SIZE          1022
#define PACKET_TS           1
#define UUS_TO_DWT_TIME     65536
// #define TS_WAIT          

# define TARGET_XTAL_OFFSET_VALUE_PPM_MIN    (-8.0f)
# define TARGET_XTAL_OFFSET_VALUE_PPM_MAX    (-6.0f)

/* The FS_XTALT_MAX_VAL defined the maximum value of the trimming value */
#define FS_XTALT_MAX_VAL                    (FS_XTALT_MASK)

/* The typical trimming range is ~48ppm over all steps, see chapter "5.14.2 Crystal Oscillator Trim" of DW1000 Datasheet */
#define AVG_TRIM_PER_PPM                    ((FS_XTALT_MAX_VAL+1)/48.0f)



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
} sender_info_t;

/*---------------------------------------------------------------------------*/

packet_t rxpkt;
packet_t txpkt;
sender_info_t instance_info;

int uCurrentTrim_val;
float xtalOffset_ppm;

static float freqMultiplier = FREQ_OFFSET_MULTIPLIER_110KB;    /* Frequency Multiplier         : Depends on .dataRate in the communication configuration */
static float hzMultiplier   = HERTZ_TO_PPM_MULTIPLIER_CHAN_2;  /* Hz to PPM transfer Multiplier: Depends on .chan in the communication configuration */


dwt_config_t config =   {
    5, /* Channel number. */
    DWT_PRF_64M, /* Pulse repetition frequency. */
    DWT_PLEN_64, /* Preamble length. Used in TX only. */
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

uint16_t node_id = 0;

/*---------------------------------------------------------------------------*/


void tx_ok_cb(const dwt_cb_data_t *cb_data){
  txpkt.seq++;
#ifdef TS_WAIT
  dwt_forcetrxoff();
  dwt_rxenable(DWT_START_RX_IMMEDIATE);
#else
  dwt_forcetrxoff();
  dwt_writetxdata(20, (uint8_t *) &txpkt, 0);
  dwt_writetxfctrl(instance_info.packet_len, 0, 0);
  if (instance_info.tx_IPI_ms != 0){

  }
  dwt_starttx(DWT_START_TX_IMMEDIATE);
#endif
}

float abs(float i){
  if (i < 0)
    return -1 * i;
  return i;
}

void rx_err_cb(const dwt_cb_data_t *cb_data){
  
  dwt_forcetrxoff();
  dwt_rxenable(DWT_START_RX_IMMEDIATE);
  printf("RX ERR: %x\n", cb_data->status);
  // printf("TX OK Sender\n");
}

void rx_ok_cb(const dwt_cb_data_t *cb_data){
  uint32_t rx_time = dwt_readrxtimestamphi32();
  dwt_forcetrxoff();
  dwt_readrxdata((uint8_t *) &rxpkt, cb_data->datalength, 0);
  int32    tmp;
  tmp = dwt_readcarrierintegrator();
  xtalOffset_ppm = tmp * (freqMultiplier * hzMultiplier);
  uint32_t tx_time = rx_time + ((UUS_TO_DWT_TIME * 5000) >> 8);
  dwt_setdelayedtrxtime(tx_time);
  dwt_writetxdata(20, (uint8_t *) &txpkt, 0);
  dwt_writetxfctrl(FRAME_SIZE, 0, 0);
  if (rxpkt.packet_type == PACKET_TS){
    dwt_writetxfctrl(FRAME_SIZE, 0, 0);
    if (dwt_starttx(DWT_START_TX_DELAYED) == DWT_SUCCESS){
      // printf("TS Frame: %d\n", rxpkt.seq);
    }else{
      printf("TX failed \n");
    }
    
  }else{
    printf("Something else\n");
  }
  printf("CLK offset: %d\n", (int) (xtalOffset_ppm * 10));
  if ((xtalOffset_ppm) < TARGET_XTAL_OFFSET_VALUE_PPM_MIN || (xtalOffset_ppm) > TARGET_XTAL_OFFSET_VALUE_PPM_MAX){
    uCurrentTrim_val -= ((TARGET_XTAL_OFFSET_VALUE_PPM_MAX + TARGET_XTAL_OFFSET_VALUE_PPM_MIN)/2 + xtalOffset_ppm) * AVG_TRIM_PER_PPM;
    uCurrentTrim_val &= FS_XTALT_MASK;
    dwt_setxtaltrim(uCurrentTrim_val);
  }
  
}


uint16_t get_node_addr(){
  uint32_t dev_id = NRF_FICR->DEVICEADDR[0];
  uint16_t node_id = 160;
  switch (dev_id){
    case 0x5270f477:
      node_id = 166;
      break;

    case 0x752a0381:
      node_id = 161;
      break;
    
    case 0x81984018:
      node_id = 165;
      break;
    
    case 0x5c50e9de:
      node_id = 168;
      break;
    
    case 0xaaf5c764:
      node_id = 162;
      break;
    
    case 0x4ed6a168:
      node_id = 170;
      break;

    case 0x25571c0e:
      node_id = 167;
      break;

    case 0x723ee061:
      node_id = 163;
      break;
    
    case 0xda82e887:
      node_id = 169;
      break;

    case 0x7605ae4e:
      node_id = 173;
      break;
    

    case 0x2510ed2a:
      node_id = 172;
      break;
    
    case 0x685c382a:
      node_id = 164;
      break;

    case 0xabe717f8:
      node_id = 171;
      break;
  };

  return node_id;
  
}


PROCESS_THREAD(range_process, ev, data)
{
  static struct etimer et;
  uint8_t irq_status;
  

  PROCESS_BEGIN();
  
  dwt_setcallbacks(&tx_ok_cb, rx_ok_cb, NULL, &rx_err_cb);

  // if(deployment_set_node_id_ieee_addr()){
  //   printf("NODE addr set successfully: %d\n", node_id);
  // }else{
  //   printf("Failed to set nodeID\n");
  // }

  node_id = get_node_addr();
  instance_info.packet_len = FRAME_SIZE;
  instance_info.tx_IPI_ms = 0;

  printf("NODE ID is: %d\n", node_id);

  switch (node_id){
  case 161:
      instance_info.tx_PC = 9;
      instance_info.packet_len = 50;
    break;
  
  case 162:
      instance_info.tx_PC = 10;
      instance_info.packet_len = 50;
    break;
  case 163:
      instance_info.tx_PC = 11;
      instance_info.packet_len = 50;
    break;
  case 164:
      instance_info.tx_PC = 12;
      instance_info.packet_len = 50;
    break;
  case 165:
      instance_info.tx_PC = 13;
      instance_info.packet_len = 50;
    break;




  case 133:
      instance_info.tx_PC = 10;
      // config.txPreambLength = DWT_PLEN_1024;
      instance_info.packet_len = 35;
    break;
  
  default:
    break;
  }
  
  instance_info.packet_len = 50;
  
  config.txCode = instance_info.tx_PC;
  dwt_configure(&config);
  dwt_configuretxrf(&txConf);
  dwt_forcetrxoff();

  txpkt.src = node_id;
  txpkt.dst = 0xffffffff;
  txpkt.seq = 0;


  
  etimer_set(&et, CLOCK_SECOND * 3);
  PROCESS_WAIT_UNTIL(etimer_expired(&et));
  dwt_writetxdata(instance_info.packet_len, (uint8_t *) &txpkt, 0);
  dwt_writetxfctrl(instance_info.packet_len, 0, 0);

#ifdef TS_WAIT
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
