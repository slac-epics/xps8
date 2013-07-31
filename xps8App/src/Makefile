TOP=../..

include $(TOP)/configure/CONFIG
#----------------------------------------
# ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

#=============================
# xps8Record.h will be created from xps8Record.dbd
DBDINC += xps8Record
DBDINC += xps8pRecord

# xps8.dbd will be created and installed
DBD    += xps8Support.dbd

LIBRARY_IOC += xps8

# Add local files
xps8_SRCS += asynOctetSocket.cpp XPS8_drivers.cpp
xps8_SRCS += XPS8.c xps8Record.cc xps8pRecord.cc

# xps8_LIBS += $(EPICS_BASE_IOC_LIBS)

#===========================

include $(TOP)/configure/RULES
#----------------------------------------
# ADD RULES AFTER THIS LINE

