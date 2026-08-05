# Dijkstra SFML Visualizer Makefile (Native Desktop + WebAssembly WebGL)

CXX = g++
EMCC = emcc

# Project Source Files
SRCS = main.cpp Button.cpp Graph.cpp GraphManager.cpp Vertex.cpp

# Directories
BUILD_DIR = build
WASM_BUILD_DIR = $(BUILD_DIR)/wasm
TARGET_PORTFOLIO_DIR = /Users/louie/Documents/GitHub/portfolio/frontend/public/wasm/sfml

# Native Desktop Flags
SFML_PREFIX ?= $(shell brew --prefix sfml@2 2>/dev/null || echo "/opt/homebrew")
NATIVE_CXXFLAGS = -std=c++17 -Wno-deprecated-declarations -I"$(SFML_PREFIX)/include"
NATIVE_LDFLAGS = -L"$(SFML_PREFIX)/lib" -lsfml-graphics -lsfml-window -lsfml-system -lsfml-audio
DESKTOP_TARGET = $(BUILD_DIR)/dijkstra-visualizer

# Emscripten WebAssembly Flags (SDL2 + SDL2_ttf WebGL backend)
EMCC_FLAGS = -s WASM=1 \
             -s USE_SDL=2 \
             -s USE_SDL_TTF=2 \
             -s INITIAL_MEMORY=67108864 \
             --preload-file fonts/ \
             -O3
WASM_TARGET = $(WASM_BUILD_DIR)/index.html

.PHONY: all wasm desktop run-desktop deploy clean help

all: wasm

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

# Build Native Desktop Executable
desktop: $(SRCS)
	@mkdir -p $(BUILD_DIR)
	$(CXX) $(NATIVE_CXXFLAGS) $(SRCS) -o $(DESKTOP_TARGET) $(NATIVE_LDFLAGS)
	@echo "Desktop build complete: $(DESKTOP_TARGET)"

# Run Native Desktop Executable
run-desktop: desktop
	./$(DESKTOP_TARGET)

# Clean Build Artifacts
clean:
	rm -rf $(BUILD_DIR)
	@echo "Cleaned build directory."

help:
	@echo "Available Makefile targets:"
	@echo "  make wasm        - Compile WebAssembly bundle to build/wasm/index.html"
	@echo "  make deploy      - Build WASM and copy assets to portfolio frontend directory"
	@echo "  make desktop     - Build native desktop binary"
	@echo "  make run-desktop - Build and execute native desktop app"
	@echo "  make clean       - Remove build artifacts"
