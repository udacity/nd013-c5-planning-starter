CARLADIR	=$(CURDIR)/../../..
BUILDDIR	=$(CURDIR)/build
BINDIR		=$(CURDIR)/bin
INSTALLDIR	=$(CURDIR)/libcarla-install
TOOLCHAIN	=$(CURDIR)/ToolChain.cmake
EIGENDIR	=$(CURDIR)/eigen-3.3.7
# BEHAVIOR_TREE_DIR = =$(CURDIR)/BehaviorTree.CPP

CPP_FILES 	:= main.cpp \
				behavior_planner_FSM.cpp \
				motion_planner.cpp \
				cubic_spiral.cpp \
				spiral_base.cpp \
				integral.cpp \
				spiral_equations.cpp \
				cost_functions.cpp \
				utils.cpp \
				vehicle_dynamic_model.cpp \
				velocity_profile_generator.cpp \
				plot_utils.cpp
				
# H_FILES   	:= $(wildcard $(CURDIR)/*.h)
# OBJ_FILES 	:= $(patsubst %.cpp,%.o,$(CPP_FILES))

CC=/usr/bin/gcc-7
CXX=/usr/bin/g++-7
CXXFLAGS=-std=c++17 -pthread -fPIC -O3 -DNDEBUG  -Wall -Wextra #-Werror

# Default to using system's default version of python
PYTHON_BIN     ?= python3
PYTHON_CONFIG  := $(PYTHON_BIN)-config
PYTHON_INCLUDE ?= $(shell $(PYTHON_CONFIG) --includes)
EXTRA_FLAGS    := $(PYTHON_INCLUDE)
# NOTE: Since python3.8, the correct invocation is `python3-config --libs --embed`.
# So of course the proper way to get python libs for embedding now is to
# invoke that, check if it crashes, and fall back to just `--libs` if it does.
LDFLAGS        += $(shell if $(PYTHON_CONFIG) --ldflags --embed >/dev/null; then $(PYTHON_CONFIG) --ldflags --embed; else $(PYTHON_CONFIG) --ldflags; fi)

# Either finds numpy or set -DWITHOUT_NUMPY
EXTRA_FLAGS     += $(shell $(PYTHON_BIN) $(CURDIR)/numpy_flags.py)
WITHOUT_NUMPY   := $(findstring $(EXTRA_FLAGS), WITHOUT_NUMPY)


ARGS += --v=0

define log
	@echo "\033[1;35m$(1)\033[0m"
endef

default: build

clean:
	@rm -rf $(BUILDDIR) $(INSTALLDIR)
	@rm -f ToolChain.cmake

run: build
	$(call log,Running C++ Client...)
	$(BINDIR)/udacity_client $(ARGS)

run.only:
	$(call log,Running C++ Client...)
	@$(BINDIR)/udacity_client $(ARGS)

build: $(BINDIR)/udacity_client build_exe

$(BINDIR)/udacity_client: | build_libcarla

build_exe:
	$(call log,Compiling C++ Client...)
	@mkdir -p $(BINDIR)
	@$(CXX) $(CXXFLAGS) -I$(INSTALLDIR)/include -I$(EIGENDIR) \
		-isystem $(INSTALLDIR)/include/system -L$(INSTALLDIR)/lib  \
		-o $(BINDIR)/udacity_client $(CPP_FILES) $(EXTRA_FLAGS) $(LDFLAGS) \
		-Wl,-Bstatic -lcarla_client -lrpc -lboost_filesystem -Wl,-Bdynamic \
		-lpng -ltiff -ljpeg -lRecast -lDetour -lDetourCrowd -lglog -lgtest 

build_libcarla: $(TOOLCHAIN)
	@cd $(CARLADIR); make setup
	@mkdir -p $(BUILDDIR)
	$(call log,Compiling LibCarla.client...)
	@{ \
		cd $(BUILDDIR); \
		if [ ! -f "build.ninja" ]; then \
		cmake \
			-G "Ninja" \
			-DCMAKE_BUILD_TYPE=Client \
			-DLIBCARLA_BUILD_RELEASE=ON \
			-DLIBCARLA_BUILD_DEBUG=OFF \
			-DLIBCARLA_BUILD_TEST=OFF \
			-DCMAKE_TOOLCHAIN_FILE=$(TOOLCHAIN) \
			-DCMAKE_INSTALL_PREFIX=$(INSTALLDIR) \
			-DCMAKE_EXPORT_COMPILE_COMMANDS=1 \
			$(CARLADIR); \
	  fi; \
		ninja; \
		ninja install | grep -v "Up-to-date:"; \
	}

$(TOOLCHAIN):
	@echo "set(CMAKE_C_COMPILER $(CC))" > $(TOOLCHAIN)
	@echo "set(CMAKE_CXX_COMPILER $(CXX))" >> $(TOOLCHAIN)
	@echo "set(CMAKE_CXX_FLAGS \"\$${CMAKE_CXX_FLAGS} $(CXXFLAGS)\" CACHE STRING \"\" FORCE)" >> $(TOOLCHAIN)