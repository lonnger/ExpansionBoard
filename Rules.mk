# ------------------------------------------------
# Rules.mk
#
# Makefile rules (based on gcc)
# ------------------------------------------------
#######################################
# binaries
#######################################
PREFIX = arm-none-eabi-
# The gcc compiler bin path can be either defined in make command via GCC_PATH variable (> make GCC_PATH=xxx)
# either it can be added to the PATH environment variable.
ifdef GCC_PATH
CC = $(GCC_PATH)/$(PREFIX)gcc
CPPC = $(GCC_PATH)/$(PREFIX)g++	
AS = $(GCC_PATH)/$(PREFIX)gcc -x assembler-with-cpp
CP = $(GCC_PATH)/$(PREFIX)objcopy
SZ = $(GCC_PATH)/$(PREFIX)size
else
CC = $(PREFIX)gcc
CPPC = $(PREFIX)g++	
AS = $(PREFIX)gcc -x assembler-with-cpp
CP = $(PREFIX)objcopy
SZ = $(PREFIX)size
endif
HEX = $(CP) -O ihex
BIN = $(CP) -O binary -S

#######################################
# CFLAGS
#######################################
# cpu
CPU = -mcpu=cortex-m3

# fpu
# NONE for Cortex-M0/M0+/M3

# float-abi

# mcu
MCU = $(CPU) -mthumb $(FPU) $(FLOAT-ABI)


# compile flags
COMPILERFLAGS = $(MCU)								# MCU

#COMPILERFLAGS += -ffreestanding						# No standard library
COMPILERFLAGS += -fdata-sections					# Place each data item into its own section
COMPILERFLAGS += -ffunction-sections				# Place each function into its own section
COMPILERFLAGS += -fstack-usage						# Generate stack usage information

COMPILERFLAGS += -specs=nano.specs					# Use newlib-nano

COMPILERFLAGS += $(OPT)								# Optimization

COMPILERFLAGS += -pipe								# Use pipes rather than temporary files

COMPILERFLAGS += -Wfatal-errors						# Stop after the first error

COMPILERFLAGS += -Wall 								# Enable all warnings
COMPILERFLAGS += -Wextra							# Enable extra warnings
COMPILERFLAGS += -Wpedantic							# Warn about anything that does not conform to the standard
COMPILERFLAGS += -Wshadow							# Warn about shadowed variables
COMPILERFLAGS += -Wdouble-promotion					# Warn about implicit conversion from float to double
COMPILERFLAGS += -Wlogical-op						# Warn about suspicious uses of logical operators
COMPILERFLAGS += -Wpointer-arith					# Warn about pointer arithmetic
COMPILERFLAGS += -Wmissing-field-initializers		# Warn about missing field initializers

COMPILERFLAGS += -Wno-unused-parameter				# Disable warning about unused parameters
COMPILERFLAGS += -Wno-unused-const-variable			# Disable warning about unused const variables

COMPILERFLAGS += -fdiagnostics-color=auto			# Enable colored diagnostics

# -gdwarf-version
# Where:
# -version
# is the DWARF format to produce. Valid values are 2, 3, and 4.
# Use a compatible debugger to load, run, and debug images. For example, ARM DS-5 Debugger is compatible with DWARF 4. Compile with the -g or -gdwarf-4 options to debug with ARM DS-5 Debugger.
# Legacy and third-party tools might not support DWARF 4 debug information. In this case you can specify the level of DWARF conformance required using the -gdwarf-2 or -gdwarf-3 options.
# Because the DWARF 4 specification supports language features that are not available in earlier versions of DWARF, the -gdwarf-2 and -gdwarf-3 options should only be used for backwards compatibility.

# Debug
ifneq ($(DEBUG), -g0)
COMPILERFLAGS += -g -gdwarf-4
endif

# Generate dependency information
COMPILERFLAGS += -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@"

ASFLAGS = $(AS_DEFS) $(AS_INCLUDES) $(COMPILERFLAGS)

CFLAGS = $(C_DEFS) $(C_INCLUDES) $(CSTD) $(COMPILERFLAGS)

CPPFLAGS = $(C_DEFS) $(C_INCLUDES) $(CPPSTD) $(COMPILERFLAGS)
CPPFLAGS += -fno-exceptions							# Disable exceptions
CPPFLAGS += -fno-rtti								# Disable rtti (eg: typeid, dynamic_cast) 
CPPFLAGS += -fno-threadsafe-statics					# Disable thread safe statics

#######################################
# LDFLAGS
#######################################
# link script

LDSCRIPT = $(LDSCRIPT_NMAE)

# libraries
LIBS = -lc -lm -lnosys 
LIBDIR = 
LDFLAGS = $(MCU) -specs=nano.specs -T$(LDSCRIPT) $(LIBDIR) $(LIBS) -Wl,-Map=$(BUILD_DIR)/$(TARGET).map,--cref -Wl,--gc-sections

# Hide details (silent mode)
ifeq ($(VERBOSE),1)
Q:=
else
Q:=@
endif

include $(CORE_MK_DIR)

BUILD_SUCCESS = @echo -e "\033[2K\033[92mBuild success!\033[m\n"

ifndef ECHO 
HIT_TOTAL != ${MAKE} ${MAKECMDGOALS} --dry-run ECHO="HIT_MARK" | grep -c "HIT_MARK"
HIT_COUNT = $(eval HIT_N != expr ${HIT_N} + 1)${HIT_N}
ECHO = @echo -e "\033[2K\033[96;49;4m[`expr ${HIT_COUNT} '*' 100 / ${HIT_TOTAL}`%]Building:\033[m$(notdir $<)\r\c"
endif
# $(info $(HIT_TOTAL) $(HIT_COUNT) $(ECHO))

# default action: build all
all: $(BUILD_DIR)/$(TARGET).elf $(BUILD_DIR)/$(TARGET).hex $(BUILD_DIR)/$(TARGET).bin
	$(ECHO) $@
	$(BUILD_SUCCESS)
	$(SZ) $(BUILD_DIR)/$(TARGET).elf

#######################################
# build the application
#######################################
# list of objects
OBJECTS = $(addprefix $(BUILD_DIR)/,$(C_SOURCES:.c=.o))
vpath %.c $(sort $(dir $(C_SOURCES)))
OBJECTS += $(addprefix $(BUILD_DIR)/,$(CPP_SOURCES:.cpp=.o))
vpath %.cpp $(sort $(dir $(CPP_SOURCES)))
OBJECTS += $(addprefix $(BUILD_DIR)/,$(CPP_COMPILED))
vpath %.s $(sort $(dir $(CPP_COMPILED)))
# list of ASM program objects
OBJECTS += $(addprefix $(BUILD_DIR)/,$(ASM_SOURCES:.s=.o))
vpath %.s $(sort $(dir $(ASM_SOURCES)))

# Print info
$(info Target: $(TARGET))
$(info Core:   $(CORE_DIR))
$(info )
$(info C sources: $(C_SOURCES))
$(info CPP sources: $(CPP_SOURCES))
$(info ASM sources: $(ASM_SOURCES))
$(info )

# Build .c files
$(BUILD_DIR)/%.o: %.c Makefile | $(BUILD_DIR) 
	$(ECHO) $@
	@mkdir -p $(dir $@)
	$(Q)$(CC) -c $(CFLAGS) -Wa,-a,-ad,-alms=$(BUILD_DIR)/$(<:.c=.lst) $< -o $@


# Build .cpp files
$(BUILD_DIR)/%.o: %.cpp Makefile | $(BUILD_DIR) 
	$(ECHO) $@
	@mkdir -p $(dir $@)
	$(Q)$(CPPC) -c $(CPPFLAGS) -Wa,-a,-ad,-alms=$(BUILD_DIR)/$(<:.cpp=.lst) $< -o $@

# Build .s files
$(BUILD_DIR)/%.o: %.s Makefile | $(BUILD_DIR)
	$(ECHO) $@
	@mkdir -p $(dir $@)
	$(Q)$(AS) -c $(ASFLAGS) $< -o $@

$(BUILD_DIR)/%.o: %.o Makefile | $(BUILD_DIR)
	$(ECHO) $@
	@mkdir -p $(dir $@)
	@cp $< $@

# Build .elf file
$(BUILD_DIR)/$(TARGET).elf: $(OBJECTS) Makefile
	$(ECHO) $@
	$(Q)$(CC) $(OBJECTS) $(LDFLAGS) -o $@

# Build .hex file
$(BUILD_DIR)/%.hex: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(ECHO) $@
	$(Q)$(HEX) $< $@
	
# Build .bin file
$(BUILD_DIR)/%.bin: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(ECHO) $@
	$(Q)$(BIN) $< $@	

# mkdir
$(BUILD_DIR):
	$(Q)mkdir -p $@

# Clean up
clean:
	@echo -e "\033[2K\033[96mCleaning up...\033[m"
	@-rm -fR $(BUILD_DIR)

# Rebuild
rebuild: 
	$(Q)$(MAKE) -s clean
	$(Q)$(MAKE) -s all

# Show size
size: $(BUILD_DIR)/$(TARGET).elf
	$(Q)$(SZ) $<


# the following filenames are not allowed
.PHONY: all clean list size rebuild


#######################################
# dependencies
#######################################
-include $(OBJECTS:%.o=%.d)
