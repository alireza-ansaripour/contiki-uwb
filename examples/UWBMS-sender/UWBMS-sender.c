



#include "callbacks_tx.h"
#define TS_WAIT             1

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


PROCESS(range_process, "Test range process");
AUTOSTART_PROCESSES(&range_process);


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
  
  uint8_t irq_status;
  

  PROCESS_BEGIN();
  
  dwt_setcallbacks(&tx_ok_cb, rx_ok_cb, NULL, &rx_err_cb);
  
  uint16_t node_id = get_node_addr();
  instance_info.packet_len = 100;
  instance_info.tx_IPI_ms = 0;

  printf("NODE ID is: %d\n", node_id);
  config.txPreambLength = DWT_PLEN_128;
  switch (node_id){
  case 160:
      instance_info.tx_PC = 10;
      instance_info.tx_wait_us = 1300;
      instance_info.tx_packet_num = 2;
      // config.txPreambLength = DWT_PLEN_1024;
    break;
  case 173:
      instance_info.tx_PC = 11;
      instance_info.tx_wait_us = 1700;
      instance_info.tx_packet_num = 2;
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
