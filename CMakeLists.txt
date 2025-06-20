cmake_minimum_required(VERSION 3.15)

set(project_name farmtrax)

project(${project_name} VERSION 0.3.0 LANGUAGES CXX)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wno-all -Wno-extra -Wno-pedantic -Wno-maybe-uninitialized -Wno-unused-variable -Wno-reorder)
endif()
cmake_policy(SET CMP0074 NEW)
cmake_policy(SET CMP0135 NEW)
set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake" ${CMAKE_MODULE_PATH})
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED On)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(params
    -Wno-reorder
)

string(TOUPPER ${project_name} project_name_upper)                                                                                                                                                                                                          
option(${project_name_upper}_BUILD_EXAMPLES "Build examples" OFF)                                                                                                                                                                                           
option(${project_name_upper}_ENABLE_TESTS "Enable tests" OFF)                                                                                                                                                                                               
include(FetchContent)

# --------------------------------------------------------------------------------------------------
set(ext_deps)

FetchContent_Declare(concord GIT_REPOSITORY https://github.com/smolfetch/concord.git GIT_TAG 2.0.0)
FetchContent_MakeAvailable(concord)
list(APPEND ext_deps concord::concord)

FetchContent_Declare(pigment GIT_REPOSITORY https://github.com/smolfetch/pigment.git GIT_TAG 0.3.0)
FetchContent_MakeAvailable(pigment)
list(APPEND ext_deps pigment::pigment)

FetchContent_Declare(entropy GIT_REPOSITORY https://github.com/smolfetch/entropy.git GIT_TAG 0.1.0)
FetchContent_MakeAvailable(entropy)
list(APPEND ext_deps entropy::entropy)

FetchContent_Declare(geoson GIT_REPOSITORY https://github.com/smolfetch/geoson.git GIT_TAG 1.1.0)
FetchContent_MakeAvailable(geoson)
list(APPEND ext_deps geoson::geoson) 

FetchContent_Declare(geotiv GIT_REPOSITORY https://github.com/smolfetch/geotiv.git GIT_TAG 1.0.0)
FetchContent_MakeAvailable(geotiv)
list(APPEND ext_deps geotiv::geotiv)

# --------------------------------------------------------------------------------------------------
add_library(${project_name} INTERFACE)
# allow users to link to this target with `target_link_libraries(mytarget farmtrax::farmtrax)`
add_library(${project_name}::${project_name} ALIAS ${project_name})
target_include_directories(${project_name} INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

install(
  DIRECTORY include/
  DESTINATION include
)
include_directories(include)

# --------------------------------------------------------------------------------------------------
include(GNUInstallDirs)

#Install headers
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

#Install and export the INTERFACE target
install(TARGETS ${project_name} EXPORT ${project_name}Targets)

#Install the export file so that `find_package(${project_name})` works
install(EXPORT ${project_name}Targets
  FILE ${project_name}Targets.cmake
  NAMESPACE ${project_name}::
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${project_name}
)

#Also export it for in‐source FetchContent usage
export(EXPORT ${project_name}Targets
  FILE "${CMAKE_CURRENT_BINARY_DIR}/${project_name}Targets.cmake"
  NAMESPACE ${project_name}::
)


# --------------------------------------------------------------------------------------------------
if(${project_name_upper}_BUILD_EXAMPLES)
  add_definitions(-DHAS_RERUN)
  set(RERUN_DOWNLOAD_AND_BUILD_ARROW OFF)
  FetchContent_Declare(rerun_sdk URL https://github.com/rerun-io/rerun/releases/latest/download/rerun_cpp_sdk.zip)
  FetchContent_MakeAvailable(rerun_sdk)
  list(APPEND ext_deps rerun_sdk)
  #
  # FetchContent_Declare(spdlog GIT_REPOSITORY https://github.com/gabime/spdlog.git GIT_TAG v1.15.0)
  # FetchContent_MakeAvailable(spdlog)
  # list(APPEND ext_deps spdlog::spdlog)
  #
  FetchContent_Declare(json URL https://github.com/nlohmann/json/releases/download/v3.12.0/json.tar.xz)
  FetchContent_MakeAvailable(json)
  list(APPEND ext_deps nlohmann_json::nlohmann_json)

  set(Boost_INCLUDE_DIR "/usr/include")
  set(sys_deps)
  find_package(Boost REQUIRED)
  include_directories(${Boost_INCLUDE_DIRS})
  list(APPEND sys_deps ${Boost_LIBRARIES})
  

  file(GLOB exe examples/*.cpp)

  # ----------------------------------------------
  set(exec_names)
  foreach(src_file IN LISTS exe)
    get_filename_component(exec_name "${src_file}" NAME_WE)
    add_executable(${exec_name} "${src_file}")
      foreach(lib_file IN LISTS internal_deps)
        target_compile_options(${exec_name} PRIVATE ${params})
        target_sources(${exec_name} PRIVATE "${lib_file}")
      endforeach()
    target_link_libraries(${exec_name} ${ext_deps})
    target_link_libraries(${exec_name} ${sys_deps})
    install(TARGETS ${exec_name} DESTINATION bin)
    list(APPEND exec_names ${exec_name})
  endforeach()
  # ----------------------------------------------
  foreach(exec IN LISTS exec_names)
    file(REMOVE "${CMAKE_CURRENT_LIST_DIR}/.execs")
    file(WRITE "${CMAKE_CURRENT_LIST_DIR}/.execs")
    get_filename_component(exec_name "${exec}" NAME)
    file(APPEND "${CMAKE_CURRENT_LIST_DIR}/.execs" "${CMAKE_CURRENT_LIST_DIR}/build/${exec_name}\n")
  endforeach()
  # ----------------------------------------------
  install(TARGETS
    ${exec_names}
    DESTINATION lib/${PROJECT_NAME}
  )
endif()


# --------------------------------------------------------------------------------------------------
if(${project_name_upper}_ENABLE_TESTS)
  enable_testing()
  add_definitions(-DENABLE_DOCTEST_IN_LIBRARY)
  include(FetchContent)
  FetchContent_Declare(
    DocTest
    GIT_REPOSITORY "https://github.com/onqtam/doctest"
  )
  FetchContent_MakeAvailable(DocTest)

  file(GLOB test_src test/*.cpp)

  foreach(src_file IN LISTS test_src)
    get_filename_component(test_name "${src_file}" NAME_WE)
    add_executable(${test_name} "${src_file}")
      foreach(lib_file IN LISTS internal_deps)
        target_compile_options(${test_name} PRIVATE ${params})
        target_sources(${test_name} PRIVATE "${lib_file}")
      endforeach()
    target_link_libraries(${test_name} ${ext_deps} doctest_with_main)
    add_test(NAME ${test_name} COMMAND ${test_name})
  endforeach()
endif()
