#include ../Makefile.tests_common

BOARD ?= avr-rss2

APPLICATION ?= tests_$(notdir $(patsubst %/,%,$(CURDIR)))

RIOTBASE ?= $(CURDIR)/../RIOT
QUIET ?= 1

USEMODULE += gnrc_netdev_default
USEMODULE += auto_init_gnrc_netif

# Ed added
#CFLAGS += -DTEST_SUITES
#CFLAGS += -DLOG_LEVEL=LOG_ALL
#CFLAGS += -DDEBUG_ASSERT_VERBOSE

# Activate ICMPv6 error messages
USEMODULE += gnrc_icmpv6_error

USEMODULE += gnrc_ipv6_router_default
USEMODULE += gnrc_ipv6_ext_frag
USEMODULE += gnrc_pktbuf_cmd
USEMODULE += gnrc_udp
USEMODULE += gnrc_rpl
USEMODULE += gnrc_rpl_srh
USEMODULE += auto_init_gnrc_rpl
# Autoinit RPL on single interface
USEMODULE += gnrc_netif_single

USEMODULE += od
USEMODULE += gnrc_icmpv6_echo
USEMODULE += shell
USEMODULE += shell_commands
USEMODULE += ps
USEMODULE += netstats_l2
USEMODULE += netstats_ipv6
USEMODULE += netstats_rpl
USEMODULE += xtimer
USEMODULE += printf_float
USEMODULE += fmt

# DEVELHELP enabled by default for all tests, set 0 to disable
DEVELHELP ?= 1

FEATURES_REQUIRED = periph_i2c
FEATURES_OPTIONAL = periph_i2c_reconfigure

# Set GNRC_PKTBUF_SIZE via CFLAGS if not being set via Kconfig.
#ifndef CONFIG_GNRC_PKTBUF_SIZE
  CFLAGS += -DCONFIG_GNRC_PKTBUF_SIZE=8192
#endif

# Set a custom channel if needed
# For KTH Contiki gateway
DEFAULT_CHANNEL=25
DEFAULT_PAN_ID=0xFEED

include $(RIOTBASE)/Makefile.include

# Set a custom channel if needed
include $(RIOTMAKE)/default-radio-settings.inc.mk

