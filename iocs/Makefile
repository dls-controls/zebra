TOP = ..
include $(TOP)/configure/CONFIG

ifeq ($(EPICS_HOST_ARCH), linux-x86)
	DIRS := $(wildcard lab test)
else ifeq ($(EPICS_HOST_ARCH), linux-x86_64)
	DIRS := $(wildcard lab test)
endif

include $(TOP)/configure/RULES_DIRS
