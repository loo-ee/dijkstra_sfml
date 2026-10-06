# Dijkstra SFML Visualizer & 3D Planetary Rover Makefile (Native Desktop + WebAssembly WebGL)

CXX = g++
EMCC = emcc

# Directories
BUILD_DIR = build
WASM_BUILD_DIR = $(BUILD_DIR)/wasm
TARGET_PORTFOLIO_DIR = /Users/louie/Documents/GitHub/portfolio/frontend/public/wasm/sfml

# 2D SFML Visualizer Sources & Target
SRCS = src/visualizer2d/main.cpp \
       src/visualizer2d/Button.cpp \
       src/visualizer2d/Graph.cpp \
       src/visualizer2d/GraphManager.cpp \
       src/visualizer2d/Vertex.cpp
DESKTOP_OBJS = $(patsubst src/visualizer2d/%.cpp,$(BUILD_DIR)/visualizer2d/%.o,$(SRCS))
DESKTOP_TARGET = $(BUILD_DIR)/dijkstra-visualizer

# Native Desktop Flags (SFML Legacy)
SFML_PREFIX ?= $(shell brew --prefix sfml@2 2>/dev/null || echo "/opt/homebrew")
NATIVE_CXXFLAGS = -std=c++17 -Wno-deprecated-declarations -Iinclude -Iinclude/visualizer2d -I"$(SFML_PREFIX)/include"
NATIVE_LDFLAGS = -L"$(SFML_PREFIX)/lib" -lsfml-graphics -lsfml-window -lsfml-system -lsfml-audio

# 3D Rover Simulation Sources & Target (Raylib 6.0+ & Jolt Physics)
ROVER_SRCS = src/rover/rover_main.cpp \
             src/rover/GraphRenderer3D.cpp \
             src/rover/TerrainHeightfield.cpp \
             src/rover/TerrainChunk.cpp \
             src/rover/ChunkManager.cpp \
             src/rover/RoverNavGraph.cpp \
             src/rover/PhysicsWorld.cpp \
             src/rover/DijkstraSolver3D.cpp \
             src/rover/PlanetaryRover.cpp \
             src/rover/RoverTelemetryHUD.cpp
ROVER_OBJS = $(patsubst src/rover/%.cpp,$(BUILD_DIR)/rover/%.o,$(ROVER_SRCS))
ROVER_TARGET = $(BUILD_DIR)/rover-simulator

RAYLIB_PREFIX ?= $(shell brew --prefix raylib 2>/dev/null || echo "/opt/homebrew")
RAYLIB_CXXFLAGS = -std=c++17 -Wall -DNDEBUG -DJPH_DEBUG_RENDERER -DJPH_OBJECT_STREAM -DJPH_PROFILE_ENABLED -DJPH_USE_CPU_COMPUTE -DJPH_USE_MTL \
                  -Iinclude -Iinclude/rover -Iexternal/JoltPhysics -I"$(RAYLIB_PREFIX)/include"
RAYLIB_LDFLAGS  = -Lexternal/JoltPhysics/Build -lJolt \
                  -L"$(RAYLIB_PREFIX)/lib" -lraylib \
                  -framework OpenGL -framework Cocoa -framework IOKit -framework CoreVideo

# Emscripten WebAssembly Flags (SDL2 + SDL2_ttf WebGL backend)
EMCC_FLAGS = -std=c++17 -Iinclude -Iinclude/visualizer2d \
             -s WASM=1 \
             -s USE_SDL=2 \
             -s USE_SDL_TTF=2 \
             -s INITIAL_MEMORY=67108864 \
             --preload-file fonts/ \
             -O3
WASM_TARGET = $(WASM_BUILD_DIR)/index.html

.PHONY: all wasm desktop run-desktop rover run-rover train-mlp deploy clean check-raylib help

all: wasm

# Pattern rule for 3D Rover object files
$(BUILD_DIR)/rover/%.o: src/rover/%.cpp
	@mkdir -p $(BUILD_DIR)/rover
	$(CXX) $(RAYLIB_CXXFLAGS) -c $< -o $@

# Pattern rule for 2D Visualizer object files
$(BUILD_DIR)/visualizer2d/%.o: src/visualizer2d/%.cpp
	@mkdir -p $(BUILD_DIR)/visualizer2d
	$(CXX) $(NATIVE_CXXFLAGS) -c $< -o $@

# Build WebAssembly WebGL Bundle
wasm: $(SRCS)
	@mkdir -p $(WASM_BUILD_DIR)
	$(EMCC) $(SRCS) -o $(WASM_TARGET) $(EMCC_FLAGS)
	@echo "WASM WebGL build complete at $(WASM_BUILD_DIR)"

# Deploy WASM assets to Portfolio frontend target directory
deploy: wasm
	@mkdir -p $(TARGET_PORTFOLIO_DIR)
	cp $(WASM_BUILD_DIR)/* $(TARGET_PORTFOLIO_DIR)/
	@echo "Deployed WASM build to $(TARGET_PORTFOLIO_DIR)"

# Build Native Desktop Executable (SFML Legacy)
desktop: $(DESKTOP_OBJS)
	@mkdir -p $(BUILD_DIR)
	$(CXX) $(DESKTOP_OBJS) -o $(DESKTOP_TARGET) $(NATIVE_LDFLAGS)
	@echo "Desktop build complete: $(DESKTOP_TARGET)"

# Run Native Desktop Executable (SFML Legacy)
run-desktop: desktop
	./$(DESKTOP_TARGET)

external/JoltPhysics/Build/libJolt.a:
	@mkdir -p external/JoltPhysics/Build
	cmake -B external/JoltPhysics/Build -S external/JoltPhysics/Build -DCMAKE_BUILD_TYPE=Release -DTARGET_UNIT_TESTS=OFF -DTARGET_HELLO_WORLD=OFF -DTARGET_PERFORMANCE_TEST=OFF -DTARGET_SAMPLES=OFF -DTARGET_VIEWER=OFF -DCMAKE_POSITION_INDEPENDENT_CODE=ON
	cmake --build external/JoltPhysics/Build -j 4

# Build Native 3D Simulation (Raylib + Jolt)
rover: external/JoltPhysics/Build/libJolt.a $(ROVER_OBJS)
	@mkdir -p $(BUILD_DIR)
	$(CXX) $(ROVER_OBJS) -o $(ROVER_TARGET) $(RAYLIB_LDFLAGS)
	@echo "Rover 3D simulation build complete: $(ROVER_TARGET)"

# Run Native 3D Simulation (Raylib)
run-rover: rover
	./$(ROVER_TARGET)

# Retrain Neural Network Traversability MLP
train-mlp:
	@/usr/bin/python3 scripts/train_mlp.py --output-header include/rover/TerrainTraversabilityMLP.h

# Clean Build Artifacts
clean:
	rm -rf $(BUILD_DIR)
	@echo "Cleaned build directory."

# Check Raylib Configuration
check-raylib:
	@echo "Checking Raylib installation at $(RAYLIB_PREFIX)..."
	@test -f "$(RAYLIB_PREFIX)/include/raylib.h" && \
		echo "✓ Raylib header found: $(RAYLIB_PREFIX)/include/raylib.h" || \
		(echo "✗ Raylib header not found! Run: brew install raylib" && exit 1)
	@test -f "$(RAYLIB_PREFIX)/lib/libraylib.dylib" -o -f "$(RAYLIB_PREFIX)/lib/libraylib.a" && \
		echo "✓ Raylib library found in $(RAYLIB_PREFIX)/lib" || \
		(echo "✗ Raylib library not found! Run: brew install raylib" && exit 1)
	@echo "✓ Raylib is ready for 3D Rover Simulation builds."

help:
	@echo "Available Makefile targets:"
	@echo "  make check-raylib - Verify Raylib installation and paths"
	@echo "  make rover        - Build native 3D simulation binary (Raylib)"
	@echo "  make run-rover    - Build and execute 3D rover simulation"
	@echo "  make train-mlp    - Train PINN traversability MLP and export C++ header"
	@echo "  make wasm         - Compile WebAssembly bundle to build/wasm/index.html"
	@echo "  make deploy       - Build WASM and copy assets to portfolio frontend directory"
	@echo "  make desktop      - Build native desktop binary (SFML legacy)"
	@echo "  make run-desktop  - Build and execute native desktop app (SFML legacy)"
	@echo "  make clean        - Remove build artifacts"
