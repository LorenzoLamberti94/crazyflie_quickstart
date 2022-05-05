# enable app support
APP=1
APP_STACKSIZE=300

VPATH += src/
PROJ_OBJ += app_main.o
# PROJ_OBJ += uart_dma_pulp.o
INCLUDES += -Iinc
# CFLAGS += -Wno-unused-variable # unused variable treated as warning and not error

CRAZYFLIE_BASE = ../crazyflie-firmware/

# Some firmware changes are required so the original files from the crazyflie-firmware are excluded
# and the modified ones are added (add one line for each)
SRC_FILES := $(filter-out $(CRAZYFLIE_BASE)/src/deck/drivers/src/multiranger.c, $(SRC_FILES))
# add folders
VPATH += ../crazyflie-firmware-modified
include $(CRAZYFLIE_BASE)/Makefile
