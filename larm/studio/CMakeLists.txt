FetchContent_Declare(lqtexec
    GIT_REPOSITORY https://github.com/lvyues1994/lqtexec.git
    GIT_TAG f9c8d69d180f5c53bada3c3f408b9c356879a2e5
    SYSTEM)
set(LQTEXEC_QT_MAJOR_VERSION 6)
set(LQTEXEC_BUILD_NETWORK OFF)
FetchContent_MakeAvailable(lqtexec)

file(GLOB_RECURSE larm_studio_sources CONFIGURE_DEPENDS src/*.cpp src/*.h)
add_library(larm_studio_ui STATIC ${larm_studio_sources})
add_library(larm::studio ALIAS larm_studio_ui)
target_include_directories(larm_studio_ui PUBLIC src)
target_link_libraries(larm_studio_ui PUBLIC larm::runtime larm::sim larm::model lqtexec::lqtexec Qt6::Widgets Qt6::OpenGL)
# Qt's keyword macros (emit, signals, slots) collide with identifiers in third-party headers.
target_compile_definitions(larm_studio_ui PUBLIC QT_NO_KEYWORDS)
larm_target_warnings(larm_studio_ui)
