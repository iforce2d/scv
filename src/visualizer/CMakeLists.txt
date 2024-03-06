cmake_minimum_required(VERSION 2.6)

project(visualizer)

set(IMGUI_DIR ../../imgui)
set(IMPLOT_DIR ../../implot)
set(SCV_DIR ../scv)

set(visualizer_SRCS
	main.cpp
	${IMGUI_DIR}/imgui.cpp
#	${IMGUI_DIR}/imgui_demo.cpp
	${IMGUI_DIR}/imgui_draw.cpp
	${IMGUI_DIR}/imgui_tables.cpp
	${IMGUI_DIR}/imgui_widgets.cpp 
	${IMGUI_DIR}/backends/imgui_impl_glfw.cpp
	${IMGUI_DIR}/backends/imgui_impl_opengl2.cpp 
#	${IMPLOT_DIR}/implot_demo.cpp
	${IMPLOT_DIR}/implot.cpp
	${IMPLOT_DIR}/implot_items.cpp
	${SCV_DIR}/planner.cpp
	${SCV_DIR}/vec3.cpp
)

include_directories (
	${IMGUI_DIR}
	${IMGUI_DIR}/backends
	${IMPLOT_DIR}
	${SCV_DIR}
)

add_executable(visualizer
	${visualizer_SRCS}
)

SET(CMAKE_CXX_FLAGS  "-lGL -lGLU")

find_package(PkgConfig REQUIRED)
pkg_check_modules(GLFW3 REQUIRED glfw3)
target_link_libraries(visualizer ${GLFW3_LIBRARIES})
