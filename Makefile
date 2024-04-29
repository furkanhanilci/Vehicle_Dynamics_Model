# Makefile for Vehicle Dynamics Simulation

CXX = g++
CXXFLAGS = -Wall -std=c++11 -O2
LDFLAGS = -lboost_python310 -lpython3.10  # Updated library name

# Specify the Python version and Boost libraries path if not standard
# You might need to adjust the Python version and path to Boost and Python libraries according to your installation
PYTHON_VERSION = 3.10
PYTHON_INCLUDE = /usr/include/python$(PYTHON_VERSION)
BOOST_INC = /usr/include
BOOST_LIB = /usr/lib/x86_64-linux-gnu  # Updated to the specific directory where Boost libraries are stored

# Include directories
INCLUDES = -I$(PYTHON_INCLUDE) -I$(BOOST_INC)

# Source and object files
SRC = vehiclesim.cpp
OBJ = $(SRC:.cpp=.o)

# Output executable
EXECUTABLE = vehicle_simulation.so

all: $(EXECUTABLE)

$(EXECUTABLE): $(OBJ)
	$(CXX) -shared -Wl,--export-dynamic $(OBJ) -L$(BOOST_LIB) $(LDFLAGS) -o $@

%.o: %.cpp
	$(CXX) $(CXXFLAGS) $(INCLUDES) -fPIC -c $< -o $@

clean:
	rm -f $(OBJ) $(EXECUTABLE)

.PHONY: all clean
