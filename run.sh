SFML_PREFIX="$(brew --prefix sfml@2)"
g++ -std=c++17 main.cpp Button.cpp Graph.cpp GraphManager.cpp Vertex.cpp \
	-I"$SFML_PREFIX/include" \
	-L"$SFML_PREFIX/lib" \
	-o build/dijkstra-visualizer \
	-lsfml-graphics -lsfml-window -lsfml-system -lsfml-audio
build/dijkstra-visualizer