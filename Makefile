BOARD ?= samr21-xpro

APPLICATION ?= tests_$(notdir $(patsubst %/,%,$(CURDIR)))

ifneq (,$(wildcard $(CURDIR)/tests/.))
  DEFAULT_MODULE += test_utils_interactive_sync
endif

ifneq (,$(filter tests_driver_%,$(APPLICATION)))
  BOARD ?= samr21-xpro
endif
BOARD ?= native
RIOTBASE ?= $(CURDIR)/../RIOT
QUIET ?= 1

# DEVELHELP enabled by default for all tests, set 0 to disable
DEVELHELP ?= 1

FEATURES_REQUIRED = periph_i2c
FEATURES_OPTIONAL = periph_i2c_reconfigure

USEMODULE += shell
USEMODULE += xtimer
USEMODULE += printf_float

include $(RIOTBASE)/Makefile.include
