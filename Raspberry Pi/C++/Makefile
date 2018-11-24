CXX = arm-linux-gnueabihf-g++.exe

COMPILE_FLAGS = -Wall -ggdb
INCLUDES = -I include/
# path #
SRC_PATH = src
BUILD_PATH = build
BIN_PATH = $(BUILD_PATH)/bin

# executable #
BIN_NAME = runner

LIBS = -lwiringPi

SRC_EXT = cpp

# .PHONY: $(BUILD_PATH)/mrf24j.o
# $(BUILD_PATH)/mrf24j.o: $(SRC_PATH)/mrf24j.cpp
#     $(CXX) -c $(SRC_PATH)/mrf24j.cpp -o $(BUILD_PATH)/mrf24j.o

# # .PHONY: $(BUILD_PATH)/main.o
# $(BUILD_PATH)/main.o: $(SRC_PATH)/main.cpp
#     $(CXX) -c $(SRC_PATH)/main.cpp -o $(BUILD_PATH)/main.o

# $(BUILD_PATH)/%.o: $(SRC_PATH)/%.$(SRC_EXT)
# 	@echo "Compiling: $< -> $@"
# 	$(CXX) $(COMPILE_FLAGS) $(INCLUDES) -MP -MMD -c $< -o $@
all: runner

.PHONY: runner
runner: mrf24j.o main.o
	$(CXX) build/mrf24j.o build/main.o -o build/bin/runner $(LIBS)

.PHONY: main.o
main.o: $(SRC_PATH)/main.cpp
	$(CXX) -c $(SRC_PATH)/main.cpp -o $(BUILD_PATH)/main.o $(INCLUDES)

# mrf24j with bcm2835
# .PHONY: mrf24j.o
# mrf24j.o: $(SRC_PATH)/mrf24j_bcm2835.cpp
# 	$(CXX) -c $(SRC_PATH)/mrf24j_bcm2835.cpp -o $(BUILD_PATH)/mrf24j.o $(INCLUDES)

# mrf24j with wiringPi
.PHONY: mrf24j.o
mrf24j.o: $(SRC_PATH)/mrf24j_wiringPi.cpp
	$(CXX) -c $(SRC_PATH)/mrf24j_wiringPi.cpp -o $(BUILD_PATH)/mrf24j.o $(INCLUDES)


