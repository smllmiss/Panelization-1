#############################################################################
# Makefile for Image Processing and Panelization
#############################################################################

CXX = clang++
CXX_FLAGS = -std=c++11 `llvm-config --cflags --ldflags --libs --system-libs` -g #-ferror-limit=1000
LIB_FLAGS = `pkg-config --libs opencv`

SOURCES = Panelize.cpp 
EXEC = Panelize

all:
	$(CXX) $(SOURCES) $(CXX_FLAGS) $(LIB_FLAGS) -o $(EXEC)

all.BestAreaFit:
	$(CXX) $(SOURCES) $(CXX_FLAGS) $(LIB_FLAGS) -o $(EXEC) 

all.BestLongSideFit:
	$(CXX) $(SOURCES) $(CXX_FLAGS) $(LIB_FLAGS) -o $(EXEC)

all.BestShortSideFit:
	$(CXX) $(SOURCES) $(CXX_FLAGS) $(LIB_FLAGS) -o $(EXEC)

all.debug:
	$(CXX) $(SOURCES) $(CXX_FLAGS) $(LIB_FLAGS) -o $(EXEC) -DDEBUG

clean:
	rm $(EXEC)

	
