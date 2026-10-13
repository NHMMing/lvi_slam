# syntax=docker/dockerfile:1
#
# lvi_slam 镜像。一个 Dockerfile 覆盖三种变体 × 两种架构：
#   ROS_DISTRO = noetic (Ubuntu 20.04) | humble (Ubuntu 22.04) | none (Ubuntu 22.04)
#   平台       = linux/amd64 | linux/arm64（Jetson Orin 等）
#
# 三个目标阶段：
#   deps   全部依赖 + GTSAM，不含本仓库代码。开发时挂载源码进去编译；
#          变化很少，推到 GHCR 后新机器直接 pull，省掉最耗时的环境搭建
#   build  在 deps 上编译本仓库并跑单元测试（测试失败则镜像构建失败）
#   app    deps + 编译产物 + 配置/工具，可直接跑 lvi_offline / roslaunch / lvi_eval
#
# 一般不直接调用 docker build，用 docker/lvi.sh image ...
# 构建上下文是仓库根目录：docker build -f docker/Dockerfile --build-arg ROS_DISTRO=noetic .

ARG ROS_DISTRO=noetic

# 按 ROS_DISTRO 选基础镜像。BuildKit 只会构建实际用到的那一个。
FROM ros:noetic-ros-base-focal AS base-noetic
FROM ros:humble-ros-base-jammy AS base-humble
FROM ubuntu:22.04 AS base-none

# ---------------------------------------------------------------------------
FROM base-${ROS_DISTRO} AS deps
ARG ROS_DISTRO
# 可选 apt 镜像，例如 http://mirrors.tuna.tsinghua.edu.cn（只替换 Ubuntu 源，不动 ROS 源）
ARG APT_MIRROR=""
ENV ROS_DISTRO=${ROS_DISTRO} \
    LANG=C.UTF-8 LC_ALL=C.UTF-8 \
    DEBIAN_FRONTEND=noninteractive \
    WS=/ws SRC=/lvi_slam
SHELL ["/bin/bash", "-o", "pipefail", "-c"]

RUN if [[ -n "$APT_MIRROR" ]]; then \
      sed -i -E "s#https?://(archive|security|ports)\.ubuntu\.com(/ubuntu-ports|/ubuntu)?#${APT_MIRROR}\2#g" \
          /etc/apt/sources.list; \
    fi
COPY docker/install_deps.sh /tmp/install_deps.sh
RUN CLEAN_APT=1 /tmp/install_deps.sh && rm /tmp/install_deps.sh
COPY docker/entrypoint.sh /usr/local/bin/lvi-entrypoint
ENTRYPOINT ["/usr/local/bin/lvi-entrypoint"]
CMD ["bash"]

# ---------------------------------------------------------------------------
FROM deps AS build
# 源码里不带 .git（见 .dockerignore），版本号由构建参数传入，写进 run_meta.json
ARG LVI_GIT_REV=unknown
ARG BUILD_TYPE=Release
COPY . /lvi_slam
RUN LVI_GIT_REV=${LVI_GIT_REV} BUILD_TYPE=${BUILD_TYPE} /lvi_slam/docker/ws_build.sh

# ---------------------------------------------------------------------------
FROM deps AS app
ARG LVI_GIT_REV=unknown
LABEL org.opencontainers.image.source="https://github.com/NHMMing/lvi_slam" \
      org.opencontainers.image.revision="${LVI_GIT_REV}"
COPY --from=build /ws/install /ws/install
COPY . /lvi_slam
WORKDIR /work
