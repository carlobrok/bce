# Add inputs and outputs from these tool invocations to the build variables
CPP_SRCS_EBC_LD := \
../pi_main/CameraCapture.cpp \
../pi_main/devices.cpp \
../pi_main/drive_calculation.cpp \
../pi_main/ebc_lane_detection.cpp \
../pi_main/img_processing.cpp \
../pi_main/line_calculation.cpp \
../pi_main/lane_data.cpp \
../pi_main/VideoServer.cpp \
../pi_main/window_box.cpp \

OBJS_EBC_LD := \
./pi_main/CameraCapture.o \
./pi_main/devices.o \
./pi_main/drive_calculation.o \
./pi_main/ebc_lane_detection.o \
./pi_main/img_processing.o \
./pi_main/line_calculation.o \
./pi_main/lane_data.o \
./pi_main/VideoServer.o \
./pi_main/window_box.o \

CPP_DEPS_EBC_LD := \
./pi_main/CameraCapture.d \
./pi_main/devices.d \
./pi_main/drive_calculation.d \
./pi_main/ebc_lane_detection.d \
./pi_main/img_processing.d \
./pi_main/line_calculation.d \
./pi_main/lane_data.d \
./pi_main/VideoServer.d \
./pi_main/window_box.d \

# Each subdirectory must supply rules for building sources it contributes
pi_main/%.o: ../pi_main/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -std=c++17 -O3 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '
