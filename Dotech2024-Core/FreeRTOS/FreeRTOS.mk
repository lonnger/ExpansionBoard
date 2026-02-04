#######################################
# Dotech2023 FreeRTOS makefile
#######################################

FREERTOS_HEAP = 0

# Sources
FreeRTOS_SRC =  \
$(FreeRTOS_PATH)/croutine.c \
$(FreeRTOS_PATH)/event_groups.c \
$(FreeRTOS_PATH)/list.c \
$(FreeRTOS_PATH)/queue.c \
$(FreeRTOS_PATH)/stream_buffer.c \
$(FreeRTOS_PATH)/tasks.c \
$(FreeRTOS_PATH)/timers.c

FreeRTOS_SRC +=  \
$(FreeRTOS_PATH)/portable/Common/mpu_wrappers.c \

ifeq ($(CPU), -mcpu=cortex-m0)
FreeRTOS_SRC += $(FreeRTOS_PATH)/portable/GCC/ARM_CM0/port.c 
else ifeq ($(CPU), -mcpu=cortex-m3)
FreeRTOS_SRC += $(FreeRTOS_PATH)/portable/GCC/ARM_CM3/port.c 
else ifeq ($(CPU), -mcpu=cortex-m4)
FreeRTOS_SRC += $(FreeRTOS_PATH)/portable/GCC/ARM_CM4F/port.c 
else ifeq ($(CPU), -mcpu=cortex-m7)
FreeRTOS_SRC += $(FreeRTOS_PATH)/portable/GCC/ARM_CM7port.c 
else
$(error "CPU not supported")
endif

ifneq ($(FREERTOS_HEAP), 0)
FreeRTOS_SRC += $(FreeRTOS_PATH)/portable/MemMang/heap_$(FREERTOS_HEAP).c
endif


# Includes
FreeRTOS_INC = -I$(FreeRTOS_PATH)/include
ifeq ($(CPU), -mcpu=cortex-m0)
FreeRTOS_INC += -I$(FreeRTOS_PATH)/portable/GCC/ARM_CM0/
else ifeq ($(CPU), -mcpu=cortex-m3)
FreeRTOS_INC += -I$(FreeRTOS_PATH)/portable/GCC/ARM_CM3/
else ifeq ($(CPU), -mcpu=cortex-m4)
FreeRTOS_INC += -I$(FreeRTOS_PATH)/portable/GCC/ARM_CM4F/
else ifeq ($(CPU), -mcpu=cortex-m7)
FreeRTOS_INC += -I$(FreeRTOS_PATH)/portable/GCC/ARM_CM7/
else
$(error "CPU not supported")
endif