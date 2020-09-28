-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include pi_main/subdir.mk
-include pi_calibrate/subdir.mk
-include objects.mk

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables

EXECUTABLES := \
	ebc_lane_detection \
	ebc_calibrate \

# All Target
all: $(EXECUTABLES)

# Tool invocations
ebc_lane_detection: $(OBJS_EBC_LD)
	@echo 'Building target: $@'
	@echo 'Invoking: GCC C++ Linker'
	g++ -o "ebc_lane_detection" $(OBJS_EBC_LD) -L /usr/local/lib/ $(LIBS)
	@echo 'Finished building target: $@'
	@echo ' '

ebc_calibrate: $(OBJS_EBC_CAL)
	@echo 'Building target: $@'
	@echo 'Invoking: GCC C++ Linker'
	g++ -o "ebc_calibrate" $(OBJS_EBC_CAL) -L /usr/local/lib/ $(LIBS)
	@echo 'Finished building target: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(EXECUTABLES)$(OBJS_EBC_LD)$(OBJS_EBC_CAL)$(CPP_DEPS_EBC_LD)$(CPP_DEPS_EBC_CAL)
	-@echo ' '

.PHONY: all clean dependents

-include ../makefile.targets
