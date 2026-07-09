CXX := g++
CXXFLAGS := -std=c++17 -Iinclude -O2 -g -Wall -Wextra
LDLIBS := -lncurses

GENERATED_DIR := bin
BUILD_DIR := build
PYTHON_MOCK_RUNNER := spark_mock_runner.py

# Source files from the library code
SRCS := $(wildcard src/*.cpp src/*/*.cpp src/*/*/*.cpp)
OBJS := $(patsubst src/%.cpp,$(BUILD_DIR)/%.o,$(SRCS))

# Example binaries
EX_SRCS := $(wildcard examples/*.cpp)
EXES := $(patsubst examples/%.cpp,$(GENERATED_DIR)/spark_%,$(EX_SRCS))

.PHONY: all clean dirs

%: dirs $(GENERATED_DIR)/spark_%
	rm -rf $(BUILD_DIR)
	@echo "Built $@"

all: dirs $(EXES) $(PYTHON_MOCK_RUNNER)
	rm -rf $(BUILD_DIR)
	cp $(PYTHON_MOCK_RUNNER) $(GENERATED_DIR)/$(PYTHON_MOCK_RUNNER)
	chmod +x $(GENERATED_DIR)/$(PYTHON_MOCK_RUNNER)
	@echo "Build successful"

dirs:
	@mkdir -p $(GENERATED_DIR)
	@mkdir -p $(BUILD_DIR)

$(BUILD_DIR)/%.o: src/%.cpp
	mkdir -p $(dir $@)
	$(CXX) $(CXXFLAGS) -c $< -o $@

$(GENERATED_DIR)/spark_%: examples/%.cpp $(OBJS)
	$(CXX) $(CXXFLAGS) $< $(OBJS) -o $@ $(LDLIBS)

clean:
	rm -rf $(GENERATED_DIR)
	rm -rf $(BUILD_DIR)