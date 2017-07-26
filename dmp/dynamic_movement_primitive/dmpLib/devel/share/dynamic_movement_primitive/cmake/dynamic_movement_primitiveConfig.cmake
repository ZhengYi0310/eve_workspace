# generated from catkin/cmake/template/pkgConfig.cmake.in

# append elements to a list and remove existing duplicates from the list
# copied from catkin/cmake/list_append_deduplicate.cmake to keep pkgConfig
# self contained
macro(_list_append_deduplicate listname)
  if(NOT "${ARGN}" STREQUAL "")
    if(${listname})
      list(REMOVE_ITEM ${listname} ${ARGN})
    endif()
    list(APPEND ${listname} ${ARGN})
  endif()
endmacro()

# append elements to a list if they are not already in the list
# copied from catkin/cmake/list_append_unique.cmake to keep pkgConfig
# self contained
macro(_list_append_unique listname)
  foreach(_item ${ARGN})
    list(FIND ${listname} ${_item} _index)
    if(_index EQUAL -1)
      list(APPEND ${listname} ${_item})
    endif()
  endforeach()
endmacro()

# pack a list of libraries with optional build configuration keywords
# copied from catkin/cmake/catkin_libraries.cmake to keep pkgConfig
# self contained
macro(_pack_libraries_with_build_configuration VAR)
  set(${VAR} "")
  set(_argn ${ARGN})
  list(LENGTH _argn _count)
  set(_index 0)
  while(${_index} LESS ${_count})
    list(GET _argn ${_index} lib)
    if("${lib}" MATCHES "^(debug|optimized|general)$")
      math(EXPR _index "${_index} + 1")
      if(${_index} EQUAL ${_count})
        message(FATAL_ERROR "_pack_libraries_with_build_configuration() the list of libraries '${ARGN}' ends with '${lib}' which is a build configuration keyword and must be followed by a library")
      endif()
      list(GET _argn ${_index} library)
      list(APPEND ${VAR} "${lib}${CATKIN_BUILD_CONFIGURATION_KEYWORD_SEPARATOR}${library}")
    else()
      list(APPEND ${VAR} "${lib}")
    endif()
    math(EXPR _index "${_index} + 1")
  endwhile()
endmacro()

# unpack a list of libraries with optional build configuration keyword prefixes
# copied from catkin/cmake/catkin_libraries.cmake to keep pkgConfig
# self contained
macro(_unpack_libraries_with_build_configuration VAR)
  set(${VAR} "")
  foreach(lib ${ARGN})
    string(REGEX REPLACE "^(debug|optimized|general)${CATKIN_BUILD_CONFIGURATION_KEYWORD_SEPARATOR}(.+)$" "\\1;\\2" lib "${lib}")
    list(APPEND ${VAR} "${lib}")
  endforeach()
endmacro()


if(dynamic_movement_primitive_CONFIG_INCLUDED)
  return()
endif()
set(dynamic_movement_primitive_CONFIG_INCLUDED TRUE)

# set variables for source/devel/install prefixes
if("TRUE" STREQUAL "TRUE")
  set(dynamic_movement_primitive_SOURCE_PREFIX /home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive)
  set(dynamic_movement_primitive_DEVEL_PREFIX /home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/dmpLib/devel)
  set(dynamic_movement_primitive_INSTALL_PREFIX "")
  set(dynamic_movement_primitive_PREFIX ${dynamic_movement_primitive_DEVEL_PREFIX})
else()
  set(dynamic_movement_primitive_SOURCE_PREFIX "")
  set(dynamic_movement_primitive_DEVEL_PREFIX "")
  set(dynamic_movement_primitive_INSTALL_PREFIX /usr/local)
  set(dynamic_movement_primitive_PREFIX ${dynamic_movement_primitive_INSTALL_PREFIX})
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "WARNING: package 'dynamic_movement_primitive' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  message("${_msg}")
endif()

# flag project as catkin-based to distinguish if a find_package()-ed project is a catkin project
set(dynamic_movement_primitive_FOUND_CATKIN_PROJECT TRUE)

if(NOT "/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/dmpLib/devel/include;/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/include;/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/dmpLib/include/;/usr/include/eigen3;/usr/include " STREQUAL " ")
  set(dynamic_movement_primitive_INCLUDE_DIRS "")
  set(_include_dirs "/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/dmpLib/devel/include;/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/include;/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/dmpLib/include/;/usr/include/eigen3;/usr/include")
  foreach(idir ${_include_dirs})
    if(IS_ABSOLUTE ${idir} AND IS_DIRECTORY ${idir})
      set(include ${idir})
    elseif("${idir} " STREQUAL "include ")
      get_filename_component(include "${dynamic_movement_primitive_DIR}/../../../include" ABSOLUTE)
      if(NOT IS_DIRECTORY ${include})
        message(FATAL_ERROR "Project 'dynamic_movement_primitive' specifies '${idir}' as an include dir, which is not found.  It does not exist in '${include}'.  Ask the maintainer 'robot <robot@todo.todo>' to fix it.")
      endif()
    else()
      message(FATAL_ERROR "Project 'dynamic_movement_primitive' specifies '${idir}' as an include dir, which is not found.  It does neither exist as an absolute directory nor in '/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/${idir}'.  Ask the maintainer 'robot <robot@todo.todo>' to fix it.")
    endif()
    _list_append_unique(dynamic_movement_primitive_INCLUDE_DIRS ${include})
  endforeach()
endif()

set(libraries "dynamic_movement_primitive;dmp++;lwr;locally_weighted_regression;/usr/lib/x86_64-linux-gnu/libboost_thread.so;/usr/lib/x86_64-linux-gnu/libboost_signals.so;/usr/lib/x86_64-linux-gnu/libboost_system.so;/usr/lib/x86_64-linux-gnu/libboost_filesystem.so;/usr/lib/x86_64-linux-gnu/libpthread.so")
foreach(library ${libraries})
  # keep build configuration keywords, target names and absolute libraries as-is
  if("${library}" MATCHES "^(debug|optimized|general)$")
    list(APPEND dynamic_movement_primitive_LIBRARIES ${library})
  elseif(TARGET ${library})
    list(APPEND dynamic_movement_primitive_LIBRARIES ${library})
  elseif(IS_ABSOLUTE ${library})
    list(APPEND dynamic_movement_primitive_LIBRARIES ${library})
  else()
    set(lib_path "")
    set(lib "${library}-NOTFOUND")
    # since the path where the library is found is returned we have to iterate over the paths manually
    foreach(path /home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/dmpLib/devel/lib;/opt/ros/indigo/lib)
      find_library(lib ${library}
        PATHS ${path}
        NO_DEFAULT_PATH NO_CMAKE_FIND_ROOT_PATH)
      if(lib)
        set(lib_path ${path})
        break()
      endif()
    endforeach()
    if(lib)
      _list_append_unique(dynamic_movement_primitive_LIBRARY_DIRS ${lib_path})
      list(APPEND dynamic_movement_primitive_LIBRARIES ${lib})
    else()
      # as a fall back for non-catkin libraries try to search globally
      find_library(lib ${library})
      if(NOT lib)
        message(FATAL_ERROR "Project '${PROJECT_NAME}' tried to find library '${library}'.  The library is neither a target nor built/installed properly.  Did you compile project 'dynamic_movement_primitive'?  Did you find_package() it before the subdirectory containing its code is included?")
      endif()
      list(APPEND dynamic_movement_primitive_LIBRARIES ${lib})
    endif()
  endif()
endforeach()

set(dynamic_movement_primitive_EXPORTED_TARGETS "dynamic_movement_primitive_generate_messages_cpp;dynamic_movement_primitive_generate_messages_lisp;dynamic_movement_primitive_generate_messages_py")
# create dummy targets for exported code generation targets to make life of users easier
foreach(t ${dynamic_movement_primitive_EXPORTED_TARGETS})
  if(NOT TARGET ${t})
    add_custom_target(${t})
  endif()
endforeach()

set(depends "cmake_modules;message_runtime;roscpp;usc_utilities;rosbag;sensor_msgs;std_msgs")
foreach(depend ${depends})
  string(REPLACE " " ";" depend_list ${depend})
  # the package name of the dependency must be kept in a unique variable so that it is not overwritten in recursive calls
  list(GET depend_list 0 dynamic_movement_primitive_dep)
  list(LENGTH depend_list count)
  if(${count} EQUAL 1)
    # simple dependencies must only be find_package()-ed once
    if(NOT ${dynamic_movement_primitive_dep}_FOUND)
      find_package(${dynamic_movement_primitive_dep} REQUIRED)
    endif()
  else()
    # dependencies with components must be find_package()-ed again
    list(REMOVE_AT depend_list 0)
    find_package(${dynamic_movement_primitive_dep} REQUIRED ${depend_list})
  endif()
  _list_append_unique(dynamic_movement_primitive_INCLUDE_DIRS ${${dynamic_movement_primitive_dep}_INCLUDE_DIRS})

  # merge build configuration keywords with library names to correctly deduplicate
  _pack_libraries_with_build_configuration(dynamic_movement_primitive_LIBRARIES ${dynamic_movement_primitive_LIBRARIES})
  _pack_libraries_with_build_configuration(_libraries ${${dynamic_movement_primitive_dep}_LIBRARIES})
  _list_append_deduplicate(dynamic_movement_primitive_LIBRARIES ${_libraries})
  # undo build configuration keyword merging after deduplication
  _unpack_libraries_with_build_configuration(dynamic_movement_primitive_LIBRARIES ${dynamic_movement_primitive_LIBRARIES})

  _list_append_unique(dynamic_movement_primitive_LIBRARY_DIRS ${${dynamic_movement_primitive_dep}_LIBRARY_DIRS})
  list(APPEND dynamic_movement_primitive_EXPORTED_TARGETS ${${dynamic_movement_primitive_dep}_EXPORTED_TARGETS})
endforeach()

set(pkg_cfg_extras "dynamic_movement_primitive-msg-extras.cmake")
foreach(extra ${pkg_cfg_extras})
  if(NOT IS_ABSOLUTE ${extra})
    set(extra ${dynamic_movement_primitive_DIR}/${extra})
  endif()
  include(${extra})
endforeach()