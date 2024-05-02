CXX = g++
CXXFLAGS = -fPIC -Wall -Wextra -O2 -std=c++11
LDFLAGS = -shared
PYTHON_VERSION = 3.8
PYTHON_INCLUDE = /usr/include/python$(PYTHON_VERSION)
BOOST_INCLUDE = /usr/include

# You might need to adjust these library paths depending on your system.
PYTHON_LIB = /usr/lib/python$(PYTHON_VERSION)
BOOST_LIB = /usr/lib

TARGET = vehiclesim.so

SOURCES = vehiclesim.cpp
OBJECTS = $(SOURCES:.cpp=.o)

# Rule to make the object files.
%.o: %.cpp
	$(CXX) $(CXXFLAGS) -I$(PYTHON_INCLUDE) -I$(BOOST_INCLUDE) -c $< -o $@

# Rule to link the final shared library.
$(TARGET): $(OBJECTS)
	$(CXX) $(LDFLAGS) -o $@ $^ -L$(BOOST_LIB) -L$(PYTHON_LIB) -lboost_python38 -lboost_numpy38 -lpython$(PYTHON_VERSION)

# Clean the build directory
clean:
	rm -f $(TARGET) $(OBJECTS)

# Rule to make everything.
all: $(TARGET)

# Declare 'clean' as not being a file.
.PHONY: clean all
