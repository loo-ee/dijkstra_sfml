# dijkstra_sfml
CSC 203 - Dijkstra Visualizer with SFML

## Install SFML on macOS

This project uses SFML headers and libraries directly from your system, so you need to install SFML before building.

1. Install Homebrew if you do not already have it.
1. Install SFML 2:

```bash
brew install sfml@2
```

1. Build the project with the Homebrew include and library paths. On Apple Silicon, use `/opt/homebrew`; on Intel Macs, use `/usr/local`.

```bash
g++ main.cpp Button.cpp Graph.cpp GraphManager.cpp Vertex.cpp \
	-I"$(brew --prefix sfml@2)/include" \
	-L"$(brew --prefix sfml@2)/lib" \
	-o build/dijkstra-visualizer \
	-lsfml-graphics -lsfml-window -lsfml-system -lsfml-audio
```

If you want, I can also update `run.sh` so it auto-detects the SFML install path and builds with one command.
