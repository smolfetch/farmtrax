cmake_minimum_required(VERSION 3.14)
project(farmtrax VERSION 0.0.1 LANGUAGES CXX)

# -----------------------------------------------------------------------------
# 1) Create the header‐only INTERFACE library
# -----------------------------------------------------------------------------
add_library(farmtrax INTERFACE)

# Allow users to link via `farmtrax::farmtrax`
add_library(farmtrax::farmtrax ALIAS farmtrax)

# -----------------------------------------------------------------------------
# 2) Public include path
# -----------------------------------------------------------------------------
# This makes <farmtrax/enu_converter.h> etc. resolve to include/farmtrax/...
target_include_directories(farmtrax
  INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# -----------------------------------------------------------------------------
# 3) Installation & Export for find_package()
# -----------------------------------------------------------------------------
include(GNUInstallDirs)

# 3a) Install headers
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

# 3b) Install and export the INTERFACE target
install(TARGETS farmtrax EXPORT concordTargets)

# 3c) Install the export file so that `find_package(farmtrax)` works
install(EXPORT concordTargets
  FILE concordTargets.cmake
  NAMESPACE farmtrax::
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/farmtrax
)

# 3d) Also export it for in‐source FetchContent usage
export(EXPORT concordTargets
  FILE "${CMAKE_CURRENT_BINARY_DIR}/concordTargets.cmake"
  NAMESPACE farmtrax::
)

