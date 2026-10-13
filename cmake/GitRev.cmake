# 构建期生成 lvi_git_rev.h（每次 make 都执行，内容不变时不改文件、不触发重编）。
# 配置期取 git 版本会过时：提交后不重新 cmake，二进制里记的还是旧版本，
# 评估结果就对不上代码了。
# 输入: SRC_DIR OUT_FILE BUILD_TYPE；环境变量 LVI_GIT_REV 优先（Docker 构建时源码里没有 .git）
if(DEFINED ENV{LVI_GIT_REV} AND NOT "$ENV{LVI_GIT_REV}" STREQUAL "")
  set(_rev "$ENV{LVI_GIT_REV}")
else()
  execute_process(
    COMMAND git describe --always --dirty --abbrev=12
    WORKING_DIRECTORY "${SRC_DIR}"
    OUTPUT_VARIABLE _rev OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_QUIET RESULT_VARIABLE _res)
  if(NOT _res EQUAL 0 OR _rev STREQUAL "")
    set(_rev "unknown")
  endif()
endif()
set(_content "#pragma once\n#define LVI_GIT_REV \"${_rev}\"\n#define LVI_BUILD_TYPE \"${BUILD_TYPE}\"\n")
if(EXISTS "${OUT_FILE}")
  file(READ "${OUT_FILE}" _old)
endif()
if(NOT "${_old}" STREQUAL "${_content}")
  file(WRITE "${OUT_FILE}" "${_content}")
endif()
