#ifndef CALLBACKS_H_

#define CALLBACKS_H_


#include "inc.h"




void rx_ok_cb(const dwt_cb_data_t *cb_data);
void rx_to_cb(const dwt_cb_data_t *cb_data);
void rx_err_cb(const dwt_cb_data_t *cb_data);
void tx_ok_cb(const dwt_cb_data_t *cb_data);

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

// void tx_ok_cb(const dwt_cb_data_t *cb_data){
//   txpkt.seq++;
// #if TS_WAIT == 1
//   if (txpkt.seq < instance_info.end_seq){
//     dwt_forcetrxoff();
//     dwt_writetxdata(20, (uint8_t *) &txpkt, 0);
//     dwt_writetxfctrl(instance_info.packet_len, 0, 0);
//     dwt_starttx(DWT_START_TX_IMMEDIATE);
//   }else{
//     dwt_forcetrxoff();
//     dwt_rxenable(DWT_START_RX_IMMEDIATE);
//   }
//   printf("TX done\n");
// #else
//   dwt_forcetrxoff();
//   dwt_writetxfctrl(instance_info.packet_len, 0, 0);
//   dwt_starttx(DWT_START_TX_IMMEDIATE);
//   dwt_writetxdata(20, (uint8_t *) &txpkt, 0);
//   printf("TX done NS\n");
// #endif
// }

void rx_err_cb(const dwt_cb_data_t *cb_data){
  dwt_forcetrxoff();
  dwt_rxenable(DWT_START_RX_IMMEDIATE);
  printf("RXERR\n");
}

#endif