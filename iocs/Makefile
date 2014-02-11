TOP = ..
include $(TOP)/configure/CONFIG

ifeq ($(EPICS_HOST_ARCH), linux-x86)
	DIRS := $(wildcard example test)
else ifeq ($(EPICS_HOST_ARCH), linux-x86_64)
	DIRS := $(wildcard example test)
else ifeq ($(EPICS_HOST_ARCH), windows-x64)
	DIRS := example_win
endif

include $(TOP)/configure/RULES_DIRS
