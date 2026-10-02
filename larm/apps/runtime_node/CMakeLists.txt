add_executable(larm_runtime_node src/main.cpp)
target_link_libraries(larm_runtime_node PRIVATE larm::ros larm::sim lrclexec::lrclexec)
larm_target_warnings(larm_runtime_node)
# Installed to lib/larm; MuJoCo is installed next to it in lib.
set_target_properties(larm_runtime_node PROPERTIES INSTALL_RPATH "$ORIGIN/..")
install(TARGETS larm_runtime_node DESTINATION lib/${PROJECT_NAME})
