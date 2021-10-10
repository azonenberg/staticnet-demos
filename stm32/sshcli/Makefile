CFLAGS=-g -mcpu=cortex-m7 -DSTM32F777 -O2
CXXFLAGS=$(CFLAGS) --std=c++17 -fno-exceptions -fno-rtti -g --specs=nano.specs \
	-I../stm32-cpp/devices/inc/ \
	-I../stm32-cpp/src/
CC=arm-none-eabi-gcc
CXX=arm-none-eabi-g++
all:
	$(CXX) *.cpp -c $(CXXFLAGS)
	$(CC) ../stm32-cpp/src/cpu/*.S -c $(CFLAGS)
	$(CXX) ../stm32-cpp/src/newlib-stubs/*.cpp -c $(CXXFLAGS)
	$(CXX) ../stm32-cpp/src/peripheral/*.cpp -c $(CXXFLAGS)
	$(CXX) ../stm32-cpp/src/util/*.cpp -c $(CXXFLAGS)
	$(CXX) ../stm32-cpp/devices/src/stm32f777.cpp -c $(CXXFLAGS)
	$(CXX) $(CXXFLAGS) *.o -Wl,-T ../stm32-cpp/devices/link/stm32f777.ld -o firmware.elf
	arm-none-eabi-objcopy -O binary --only-section=.text --only-section=.data firmware.elf firmware.bin
	./imagesize.sh
