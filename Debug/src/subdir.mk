################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/ForwardKinematics.cpp \
../src/IK6AxisInline.cpp \
../src/IK6AxisOffset.cpp \
../src/Jacobian.cpp 

CPP_DEPS += \
./src/ForwardKinematics.d \
./src/IK6AxisInline.d \
./src/IK6AxisOffset.d \
./src/Jacobian.d 

OBJS += \
./src/ForwardKinematics.o \
./src/IK6AxisInline.o \
./src/IK6AxisOffset.o \
./src/Jacobian.o 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.cpp src/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -O0 -g3 -Wall -c -fmessage-length=0 -fPIC -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


clean: clean-src

clean-src:
	-$(RM) ./src/ForwardKinematics.d ./src/ForwardKinematics.o ./src/IK6AxisInline.d ./src/IK6AxisInline.o ./src/IK6AxisOffset.d ./src/IK6AxisOffset.o ./src/Jacobian.d ./src/Jacobian.o

.PHONY: clean-src

