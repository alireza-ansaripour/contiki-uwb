# CONTIKI_PROJECT = rng
all: rng





# enable softdevice
#NRF52_SOFTDEVICE = 132

# print over NRF RTT
#PRINTF_OVER_RTT = 1

# enable nRF log over RTT
NRF_LOG_OVER_RTT = 1

CFLAGS += -DNRF_SHOW_RESETREASON

DEFINES+=PROJECT_CONF_H=\"project-conf.h\"
callbacks.$(TARGET):$(OBJECTDIR)/blink-led.o


# Enable Rime stack
CONTIKI_WITH_IPV4 = 0
CONTIKI_WITH_IPV6 = 0
CONTIKI_WITH_RIME = 1

UWB_CONTIKI ?= ../..
include $(UWB_CONTIKI)/Makefile.uwb
$(info info: $(TARGETDIRS))