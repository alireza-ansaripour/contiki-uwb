
#include "callbacks.h"

#define TS_WAIT             1


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

  PROCESS_BEGIN();
  
  dwt_setcallbacks(NULL, &rx_ok_cb, &rx_to_cb, &rx_err_cb);
  uint16_t node_id = get_node_addr();
  printf("NODE ID is: %d\n", node_id);


  switch (node_id){    
    case 166:
        rx_info.rxCode = 10;
        rx_info.rx_wait_us = 1200;
      break;
    case 167:
        rx_info.rxCode = 11;
        rx_info.rx_wait_us = 1800;
      break;
    

    default:
      break;
  }


#if TS_WAIT == 1
  rx_info.ts_rxCode = 9;
#else
  rx_info.ts_rxCode = config.rxCode;
  config.rxCode = rx_info.rxCode;
#endif

  dwt_configure(&config);
  bytes_received = 0;
  dwt_forcetrxoff();
  dwt_rxenable(DWT_START_RX_IMMEDIATE);
  while (1){
    etimer_set(&et, CLOCK_SECOND);
    PROCESS_WAIT_UNTIL(etimer_expired(&et));
    if (last_seq != 0){
      // printf("PRR: %d\n", (100 * packets_received) / (last_seq - start_seq));
      start_seq = last_seq;
      packets_received = 0;
    }else{
      printf("No data yet!!!\n");
    }
  }
  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
