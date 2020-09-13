# Add inputs and outputs from these tool invocations to the build variables
CPP_SRCS += \
../pi_src/ebc_lane_detection.cpp \
../pi_src/CameraCapture.cpp \
../pi_src/VideoServer.cpp \
../pi_src/devices.cpp \
../pi_src/img_processing.cpp \
../pi_src/line_calculation.cpp \
../pi_src/drive_calculation.cpp \

OBJS += \
./pi_src/ebc_lane_detection.o \
./pi_src/CameraCapture.o \
./pi_src/VideoServer.o \
./pi_src/devices.o \
./pi_src/img_processing.o \
./pi_src/line_calculation.o \
./pi_src/drive_calculation.o \

CPP_DEPS += \
./pi_src/ebc_lane_detection.d \
./pi_src/CameraCapture.d \
./pi_src/VideoServer.d \
./pi_src/devices.d \
./pi_src/img_processing.d \
./pi_src/line_calculation.d \
./pi_src/drive_calculation.d \

# Each subdirectory must supply rules for building sources it contributes
pi_src/%.o: ../pi_src/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -std=c++17 -O3 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '
