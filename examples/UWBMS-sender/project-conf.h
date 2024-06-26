#ifndef PROJECT_CONF_H_
#define PROJECT_CONF_H_

#define LINKADDR_CONF_SIZE 2
//#define LINKADDR_CONF_SIZE 8
//#define DWM1001_USE_BT_ADDR_FOR_UWB 1

/* use the following to optimise ranging delays for EVB1000 */
//#define DW1000_CONF_EXTREME_RNG_TIMING 1

#define APP_RADIO_CONF 1

#if APP_RADIO_CONF == 1
#define DW1000_CONF_CHANNEL        3
#define DW1000_CONF_PRF            DWT_PRF_64M
#define DW1000_CONF_PLEN           DWT_PLEN_1024
#define DW1000_CONF_PAC            DWT_PAC32
#define DW1000_CONF_SFD_MODE       0
#define DW1000_CONF_DATA_RATE      DWT_BR_6M8
#define DW1000_CONF_PHR_MODE       DWT_PHRMODE_STD
#define DW1000_CONF_PREAMBLE_CODE  9
#define DW1000_CONF_SFD_TIMEOUT    (2000 + 8 - 8)
#define DW1000_CONF_TX_POWER       0x1F1F1F1F //0x08080808

#elif APP_RADIO_CONF == 2
#define DW1000_CONF_CHANNEL        2
#define DW1000_CONF_PRF            DWT_PRF_64M
#define DW1000_CONF_PLEN           DWT_PLEN_1024
#define DW1000_CONF_PAC            DWT_PAC32
#define DW1000_CONF_SFD_MODE       1
#define DW1000_CONF_DATA_RATE      DWT_BR_110K
#define DW1000_CONF_PHR_MODE       DWT_PHRMODE_STD
#define DW1000_CONF_PREAMBLE_CODE  9
#define DW1000_CONF_SFD_TIMEOUT    (1025 + 64 - 32)

#else
#error App: radio config is not set
#endif

#endif /* PROJECT_CONF_H_ */
