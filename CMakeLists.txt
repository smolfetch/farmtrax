cmake_minimum_required(VERSION 3.14)
project(farmtrax VERSION 0.0.1 LANGUAGES CXX)

# -----------------------------------------------------------------------------
# 1) Pull in concord dependency
# -----------------------------------------------------------------------------
include(FetchContent)
FetchContent_Declare(
  concord
  GIT_REPOSITORY https://github.com/bresilla/concord.git
  GIT_TAG        main
)
FetchContent_MakeAvailable(concord)

# -----------------------------------------------------------------------------
# 2) Create the header‐only INTERFACE library
# -----------------------------------------------------------------------------
add_library(farmtrax INTERFACE)

# Allow users to link via `farmtrax::farmtrax`
add_library(farmtrax::farmtrax ALIAS farmtrax)

# -----------------------------------------------------------------------------
# 3) Public include path
# -----------------------------------------------------------------------------
target_include_directories(farmtrax
  INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# -----------------------------------------------------------------------------
# 4) Link dependencies
# -----------------------------------------------------------------------------
target_link_libraries(farmtrax
  INTERFACE
    concord::concord
)

# -----------------------------------------------------------------------------
# 5) Installation & Export for find_package()
# -----------------------------------------------------------------------------
include(GNUInstallDirs)

# 5a) Install headers
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

# 5b) Install and export the INTERFACE target
install(TARGETS farmtrax EXPORT concordTargets)

# 5c) Install the export file so that `find_package(farmtrax)` works
install(EXPORT concordTargets
  FILE concordTargets.cmake
  NAMESPACE farmtrax::
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/farmtrax
)

# 5d) Also export it for in‐source FetchContent usage
export(EXPORT concordTargets
  FILE "${CMAKE_CURRENT_BINARY_DIR}/concordTargets.cmake"
  NAMESPACE farmtrax::
)

