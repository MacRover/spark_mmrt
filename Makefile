CXX := g++
CXXFLAGS := -std=c++17 -Iinclude -O2 -g -Wall -Wextra
LDLIBS := -lncurses

GENERATED_DIR := bin
BUILD_DIR := build

# Source files from the library code
SRCS := $(wildcard src/*.cpp src/*/*.cpp src/*/*/*.cpp)
OBJS := $(patsubst src/%.cpp,$(BUILD_DIR)/%.o,$(SRCS))

# Example binaries
EX_SRCS := $(wildcard examples/*.cpp)
EXES := $(patsubst examples/%.cpp,$(GENERATED_DIR)/%,$(EX_SRCS))

.PHONY: all clean dirs

%: dirs $(GENERATED_DIR)/%
	rm -rf $(BUILD_DIR)
	@echo "Built $@"

all: dirs $(EXES)
	rm -rf $(BUILD_DIR)

dirs:
	mkdir -p $(GENERATED_DIR)
	mkdir -p $(BUILD_DIR)

$(BUILD_DIR)/%.o: src/%.cpp
	mkdir -p $(dir $@)
	$(CXX) $(CXXFLAGS) -c $< -o $@

$(GENERATED_DIR)/%: examples/%.cpp $(OBJS)
	$(CXX) $(CXXFLAGS) $< $(OBJS) -o $@ $(LDLIBS)

clean:
	rm -rf $(GENERATED_DIR)
