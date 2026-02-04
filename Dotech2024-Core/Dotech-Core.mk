#######################################
# Dotech2023-Core defines
#######################################
FreeRTOS_PATH = $(CORE_DIR)/FreeRTOS
SystemView_PATH = $(CORE_DIR)/Diagnostic/SystemView

#######################################
# Dotech2023-Core includes
#######################################
include $(FreeRTOS_PATH)/FreeRTOS.mk
include $(SystemView_PATH)/SystemView.mk


#######################################
# Sources
#######################################
# C Sources
C_SRC = \
$(wildcard $(CORE_DIR)/Drivers/LORA/*.c)

# C++ Sources
CPP_SRC =  \
$(FreeRTOS_PATH)/os.cpp \
$(wildcard $(CORE_DIR)/Diagnostic/*.cpp) \
$(wildcard $(CORE_DIR)/Drivers/*.cpp) \
$(wildcard $(CORE_DIR)/Utils/*.cpp) \
$(wildcard $(CORE_DIR)/Control/*.cpp) \
$(wildcard $(CORE_DIR)/Drivers/LORA/*.cpp) \

# ASM Sources
AS_SRC = 


#######################################
# Defines
#######################################
# ASM Defines
AS_DEF =

# Defines
DEFS = \
-DTRUE=1 \
-DFALSE=0

#######################################
# Includes
#######################################
# ASM Includes
AS_INC =

# Includes
INC =  \
-I$(CORE_DIR)/Communication \
-I$(CORE_DIR)/Control \
-I$(CORE_DIR)/Diagnostic \
-I$(CORE_DIR)/Drivers  \
-I$(CORE_DIR)/Utils \
-I$(CORE_DIR)/Drivers/LORA \
-I$(CORE_DIR)/Drivers/matrix  \


#######################################
# Add to makefile
#######################################
C_SOURCES += $(C_SRC) $(FreeRTOS_SRC) $(SystemView_SRC)

CPP_SOURCES += $(CPP_SRC)

ASM_SOURCES += $(AS_SRC) $(SystemView_AS_SRC)

AS_DEFS += $(AS_DEF)

C_DEFS += $(DEFS) -DAPP_NAME=$(TARGET)

AS_INCLUDES += $(AS_INC)

C_INCLUDES += $(INC) $(FreeRTOS_INC) $(SystemView_INC)
