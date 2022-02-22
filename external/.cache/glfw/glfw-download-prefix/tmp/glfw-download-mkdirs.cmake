# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "D:/repos/EngineForAnimationCourse-master/cmake/../external/glfw"
  "E:/Repositories/igl_playtesting/glfw-build"
  "D:/repos/EngineForAnimationCourse-master/external/.cache/glfw/glfw-download-prefix"
  "D:/repos/EngineForAnimationCourse-master/external/.cache/glfw/glfw-download-prefix/tmp"
  "D:/repos/EngineForAnimationCourse-master/external/.cache/glfw/glfw-download-prefix/src/glfw-download-stamp"
  "D:/repos/EngineForAnimationCourse-master/external/.cache/glfw/glfw-download-prefix/src"
  "D:/repos/EngineForAnimationCourse-master/external/.cache/glfw/glfw-download-prefix/src/glfw-download-stamp"
)

set(configSubDirs Debug;Release;MinSizeRel;RelWithDebInfo)
foreach(subDir IN LISTS configSubDirs)
  file(MAKE_DIRECTORY "D:/repos/EngineForAnimationCourse-master/external/.cache/glfw/glfw-download-prefix/src/glfw-download-stamp/${subDir}")
endforeach()
