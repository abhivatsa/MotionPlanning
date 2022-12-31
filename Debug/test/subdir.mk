################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../test/ForwardKinematics_test.cpp \
../test/IK6AxisInline_test.cpp \
../test/IK6AxisOffset_test.cpp \
../test/Jacobian_test.cpp 

CPP_DEPS += \
./test/ForwardKinematics_test.d \
./test/IK6AxisInline_test.d \
./test/IK6AxisOffset_test.d \
./test/Jacobian_test.d 

OBJS += \
./test/ForwardKinematics_test.o \
./test/IK6AxisInline_test.o \
./test/IK6AxisOffset_test.o \
./test/Jacobian_test.o 


# Each subdirectory must supply rules for building sources it contributes
test/%.o: ../test/%.cpp test/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -I"/home/abhishek/eclipse-workspace/MotionPlanning/include" -O0 -g3 -Wall -c -fmessage-length=0 -fPIC -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


clean: clean-test

clean-test:
	-$(RM) ./test/ForwardKinematics_test.d ./test/ForwardKinematics_test.o ./test/IK6AxisInline_test.d ./test/IK6AxisInline_test.o ./test/IK6AxisOffset_test.d ./test/IK6AxisOffset_test.o ./test/Jacobian_test.d ./test/Jacobian_test.o

.PHONY: clean-test

