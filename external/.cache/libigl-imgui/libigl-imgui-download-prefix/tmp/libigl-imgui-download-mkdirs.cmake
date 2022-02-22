# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "D:/repos/EngineForAnimationCourse-master/cmake/../external/libigl-imgui"
  "E:/Repositories/igl_playtesting/libigl-imgui-build"
  "D:/repos/EngineForAnimationCourse-master/external/.cache/libigl-imgui/libigl-imgui-download-prefix"
  "D:/repos/EngineForAnimationCourse-master/external/.cache/libigl-imgui/libigl-imgui-download-prefix/tmp"
  "D:/repos/EngineForAnimationCourse-master/external/.cache/libigl-imgui/libigl-imgui-download-prefix/src/libigl-imgui-download-stamp"
  "D:/repos/EngineForAnimationCourse-master/external/.cache/libigl-imgui/libigl-imgui-download-prefix/src"
  "D:/repos/EngineForAnimationCourse-master/external/.cache/libigl-imgui/libigl-imgui-download-prefix/src/libigl-imgui-download-stamp"
)

set(configSubDirs Debug;Release;MinSizeRel;RelWithDebInfo)
foreach(subDir IN LISTS configSubDirs)
  file(MAKE_DIRECTORY "D:/repos/EngineForAnimationCourse-master/external/.cache/libigl-imgui/libigl-imgui-download-prefix/src/libigl-imgui-download-stamp/${subDir}")
endforeach()
