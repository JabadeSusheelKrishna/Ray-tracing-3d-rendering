cmake_minimum_required(VERSION 3.18)
set(CMAKE_BUILD_TYPE Release)

project(cs7302 
  VERSION 1.0 
  LANGUAGES C CXX
)

if (APPLE)
	set(CMAKE_MACOSX_RPATH ON)
endif()

if (MSVC)
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /D_CRT_SECURE_NO_WARNINGS")
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /MP")
endif()

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

###############################################################################
# Everything else
###############################################################################

add_subdirectory(extern/tinyobjloader)
add_subdirectory(extern/tinyexr)
add_subdirectory(extern/json)

include_directories(
	headers/
	extern/
	extern/tinyexr/deps/miniz/
)

###############################################################################
# Main executable
###############################################################################

add_executable(render
	render.cpp

	bsdf.cpp
	camera.cpp
	light.cpp
	scene.cpp
	surface.cpp
	texture.cpp

	# DEPS
  	extern/tinyexr/deps/miniz/miniz.c
)

target_link_libraries(render
	PRIVATE nlohmann_json::nlohmann_json
)
