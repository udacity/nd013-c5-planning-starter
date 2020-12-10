project(MPC)

cmake_minimum_required (VERSION 3.10)

# -g allows for gdb debugging
# turn on -03 for best performance
add_definitions(-std=c++14 -O3)

set(CXX_FLAGS "-pthread -fPIC -DNDEBUG -Werror -Wall -Wextra")
set(CMAKE_CXX_FLAGS "${CXX_FLAGS} -Wl,-Bstatic -lcarla_client -lrpc -lboost_filesystem -Wl,-Bdynamic")

set(sources MPC.cpp main.cpp)

include_directories(/usr/local/include)
link_directories(/usr/local/lib)

include_directories(eigen-3.3.7)

include_directories(libcarla-install/include)

link_directories(libcarla-install/lib)
include_directories(libcarla-install/include)

include_directories(../../Build/boost-1.72.0-c8-install/include)
link_directories(../../Build/boost-1.72.0-c8-install/lib)

include_directories(../../Build/rpclib-v2.2.1_c3-c8-libcxx-install/include)
link_directories(../../Build/rpclib-v2.2.1_c3-c8-libcxx-install/lib)

include_directories(../../Build/recast-cdce4e-c8-install/include)
link_directories(../../Build/recast-cdce4e-c8-install/lib)

include_directories(../../Build/)
link_directories(../../Build/)

if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")

endif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")

add_executable(carla_mpc ${sources})

target_link_libraries(carla_mpc ipopt)