# Add inputs and outputs from these tool invocations to the build variables
CPP_SRCS_EBC_CAL := \
../pi_calibrate/ebc_lane_detection.cpp \
../pi_calibrate/VideoClient.cpp \

OBJS_EBC_CAL := \
./pi_calibrate/ebc_lane_detection.o \
./pi_calibrate/VideoClient.o \

CPP_DEPS_EBC_CAL := \
./pi_calibrate/ebc_lane_detection.d \
./pi_calibrate/VideoClient.d \

# Each subdirectory must supply rules for building sources it contributes
pi_calibrate/%.o: ../pi_calibrate/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -std=c++17 -O3 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '
