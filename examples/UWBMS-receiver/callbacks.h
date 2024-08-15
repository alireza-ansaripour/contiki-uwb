#ifndef _RX_CALLBACKS
#define _RX_CALLBACKS


#include "contiki.h"
#include "lib/random.h"
#include "net/rime/rime.h"
#include "leds.h"
#include "net/netstack.h"
#include <stdio.h>
#include "dw1000.h"
#include "dw1000-ranging.h"
#include <sys/node-id.h>
#include "net/netstack.h"
#include "core/net/linkaddr.h"
/*---------------------------------------------------------------------------*/
PROCESS(range_process, "Test range process");
AUTOSTART_PROCESSES(&range_process);
#define UUS_TO_DWT_TIME           65536
#define TS_WAIT                   0
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
  uint8_t rxCode;
  uint8_t ts_rxCode;
  uint16_t rx_wait_us;
} rx_info_t;


/*---------------------------------------------------------------------------*/
packet_t rxpkt;

dwt_config_t config =   {
    5, /* Channel number. */
    DWT_PRF_64M, /* Pulse repetition frequency. */
    DWT_PLEN_1024, /* Preamble length. Used in TX only. */
    DWT_PAC8, /* Preamble acquisition chunk size. Used in RX only. */
    9, /* TX preamble code. Used in TX only. */
    9, /* RX preamble code. Used in RX only. */
    1, /* 0 to use standard SFD, 1 to use non-standard SFD. */
    DWT_BR_6M8, /* Data rate. */
    DWT_PHRMODE_EXT, /* PHY header mode. */
    (8000 + 1 + 64 - 64) /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
};
int bytes_received = 0;

uint32_t last_seq = 0;
uint32_t start_seq = 0;

int packets_received = 0;
rx_info_t rx_info;
/*---------------------------------------------------------------------------*/

void rx_ok_cb(const dwt_cb_data_t *cb_data){
  dwt_readrxdata((uint8_t *) &rxpkt, 20, 0);
  uint32_t rx_timestamp;
  uint32_t tx_time;
  rx_timestamp = dwt_readrxtimestamphi32();
  dwt_forcetrxoff();
  
  // if (rxpkt.seq - last_seq != 1)
  //   printf("Diff: %d, %d\n", rxpkt.seq, rxpkt.seq - last_seq);
  if (rxpkt.packet_type == 1){
    config.rxCode = rx_info.rxCode;
    dwt_configure(&config);
    dwt_setpreambledetecttimeout(1000);
    dwt_rxenable(DWT_START_RX_IMMEDIATE);
    printf("TS received\n");
  }
  
  if (rxpkt.packet_type == 2){
    last_seq = rxpkt.seq;
    packets_received++;
    dwt_rxenable(DWT_START_RX_IMMEDIATE);
    printf("DA RX\n");
  }
  
}

void rx_to_cb(const dwt_cb_data_t *cb_data){
  config.rxCode = rx_info.ts_rxCode;
  dwt_configure(&config);
  dwt_setpreambledetecttimeout(0);
  dwt_rxenable(DWT_START_RX_IMMEDIATE);
  printf("TO\n");
}

void rx_err_cb(const dwt_cb_data_t *cb_data){
  dwt_forcetrxoff();
  config.rxCode = rx_info.ts_rxCode;
  dwt_configure(&config);
  dwt_rxenable(DWT_START_RX_IMMEDIATE);
  printf("RX ERR: %d\n", cb_data->status);
}



#endif