# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "D:/repos/EngineForAnimationCourse-master/cmake/../external/stb"
  "E:/Repositories/igl_playtesting/stb-build"
  "D:/repos/EngineForAnimationCourse-master/external/.cache/stb/stb-download-prefix"
  "D:/repos/EngineForAnimationCourse-master/external/.cache/stb/stb-download-prefix/tmp"
  "D:/repos/EngineForAnimationCourse-master/external/.cache/stb/stb-download-prefix/src/stb-download-stamp"
  "D:/repos/EngineForAnimationCourse-master/external/.cache/stb/stb-download-prefix/src"
  "D:/repos/EngineForAnimationCourse-master/external/.cache/stb/stb-download-prefix/src/stb-download-stamp"
)

set(configSubDirs Debug;Release;MinSizeRel;RelWithDebInfo)
foreach(subDir IN LISTS configSubDirs)
  file(MAKE_DIRECTORY "D:/repos/EngineForAnimationCourse-master/external/.cache/stb/stb-download-prefix/src/stb-download-stamp/${subDir}")
endforeach()
