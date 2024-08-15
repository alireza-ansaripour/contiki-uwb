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
#include "callbacks.h"

PROCESS(range_process, "Test range process");
AUTOSTART_PROCESSES(&range_process);

PROCESS_THREAD(range_process, ev, data)
{
  
  uint8_t irq_status;
  

  PROCESS_BEGIN();
  
  dwt_setcallbacks(&tx_ok_cb, rx_ok_cb, NULL, &rx_err_cb);
  uint16_t node_id = 0;
  // if(deployment_set_node_id_ieee_addr()){
  //   printf("NODE addr set successfully: %d\n", node_id);
  // }else{
  //   printf("Failed to set nodeID\n");
  // }
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
