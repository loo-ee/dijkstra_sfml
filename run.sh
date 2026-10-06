#!/bin/bash
SFML_PREFIX="$(brew --prefix sfml@2)"
g++ -std=c++17 -Wno-deprecated-declarations \
	src/visualizer2d/main.cpp \
	src/visualizer2d/Button.cpp \
	src/visualizer2d/Graph.cpp \
	src/visualizer2d/GraphManager.cpp \
	src/visualizer2d/Vertex.cpp \
	-Iinclude -Iinclude/visualizer2d \
	-I"$SFML_PREFIX/include" \
	-L"$SFML_PREFIX/lib" \
	-o build/dijkstra-visualizer \
	-lsfml-graphics -lsfml-window -lsfml-system -lsfml-audio
build/dijkstra-visualizer