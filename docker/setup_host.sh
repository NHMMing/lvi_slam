#!/usr/bin/env bash
# 宿主机一次性准备：安装 Docker Engine（+ buildx/compose 插件），可选 NVIDIA 容器运行时、
# arm64 模拟器。新机器只需要跑这一次，之后所有依赖都在镜像里。
#
# 用法（需要 root；-E 保留当前 shell 的代理变量，没有代理时可省略）:
#   sudo -E docker/setup_host.sh                # Docker + （检测到 NVIDIA 显卡时）nvidia-container-toolkit
#   sudo -E docker/setup_host.sh --arm64        # 另外注册 arm64 模拟器，用于本机交叉构建 Orin 镜像
#   sudo -E docker/setup_host.sh --no-nvidia    # 跳过 NVIDIA 运行时
#
# 代理：检测到 http(s)_proxy 时会同时写入 dockerd 的 systemd 配置——
# dockerd 是系统服务，不继承登录 shell 的环境变量，不配就拉不动镜像。
#
# 支持 Ubuntu 20.04 / 22.04 / 24.04（x86_64 与 aarch64）。
# Jetson（JetPack 5/6）自带 Docker 和 NVIDIA 运行时，只需要把用户加入 docker 组：
#   sudo usermod -aG docker $USER
set -euo pipefail

WITH_NVIDIA=auto
WITH_ARM64=0
for a in "$@"; do
  case "$a" in
    --no-nvidia) WITH_NVIDIA=0 ;;
    --nvidia)    WITH_NVIDIA=1 ;;
    --arm64)     WITH_ARM64=1 ;;
    -h|--help)   sed -n '2,17p' "$0" | sed 's/^# \{0,1\}//'; exit 0 ;;
    *) echo "未知参数: $a" >&2; exit 1 ;;
  esac
done

[[ $EUID -eq 0 ]] || { echo "请用 sudo -E 运行" >&2; exit 1; }
TARGET_USER="${SUDO_USER:-}"
log() { echo -e "\033[1;32m[setup]\033[0m $*"; }

if [[ -f /etc/nv_tegra_release ]]; then
  log "检测到 Jetson：JetPack 已自带 Docker 与 NVIDIA 运行时，只做用户组配置"
  [[ -n "$TARGET_USER" ]] && usermod -aG docker "$TARGET_USER"
  exit 0
fi

PROXY="${https_proxy:-${HTTPS_PROXY:-${http_proxy:-${HTTP_PROXY:-}}}}"
NOPROXY="${no_proxy:-${NO_PROXY:-localhost,127.0.0.1}}"
[[ -n "$PROXY" ]] && log "使用代理 $PROXY"

# ---- 1. Docker Engine（官方 apt 源）----
. /etc/os-release
if ! command -v docker >/dev/null || ! docker buildx version >/dev/null 2>&1; then
  log "安装 Docker Engine（$VERSION_CODENAME）"
  apt-get update
  apt-get install -y ca-certificates curl gnupg
  install -m 0755 -d /etc/apt/keyrings
  curl -fsSL https://download.docker.com/linux/ubuntu/gpg -o /etc/apt/keyrings/docker.asc
  chmod a+r /etc/apt/keyrings/docker.asc
  echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.asc] \
https://download.docker.com/linux/ubuntu $VERSION_CODENAME stable" \
    > /etc/apt/sources.list.d/docker.list
  apt-get update
  apt-get install -y docker-ce docker-ce-cli containerd.io \
                     docker-buildx-plugin docker-compose-plugin
else
  log "Docker 已安装：$(docker --version)"
fi

# ---- 2. dockerd 代理 ----
if [[ -n "$PROXY" ]]; then
  mkdir -p /etc/systemd/system/docker.service.d
  cat > /etc/systemd/system/docker.service.d/http-proxy.conf <<EOF
[Service]
Environment="HTTP_PROXY=$PROXY"
Environment="HTTPS_PROXY=$PROXY"
Environment="NO_PROXY=$NOPROXY"
EOF
  log "已写入 dockerd 代理配置"
fi

# ---- 3. NVIDIA 容器运行时（只影响容器里 RViz 等 GUI 的 GPU 渲染，算法本身不需要 GPU）----
if [[ "$WITH_NVIDIA" == auto ]]; then
  command -v nvidia-smi >/dev/null && WITH_NVIDIA=1 || WITH_NVIDIA=0
fi
if [[ "$WITH_NVIDIA" == 1 ]]; then
  log "安装 nvidia-container-toolkit"
  curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey |
    gpg --dearmor --yes -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg
  curl -fsSL https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list |
    sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' \
    > /etc/apt/sources.list.d/nvidia-container-toolkit.list
  apt-get update
  apt-get install -y nvidia-container-toolkit
  nvidia-ctk runtime configure --runtime=docker
fi

systemctl daemon-reload
systemctl enable --now docker
systemctl restart docker

# ---- 4. 用户组：免 sudo 使用 docker ----
if [[ -n "$TARGET_USER" ]]; then
  usermod -aG docker "$TARGET_USER"
  log "已把 $TARGET_USER 加入 docker 组（重新登录或执行 newgrp docker 后生效）"
fi

# ---- 5. arm64 模拟器（可选）：x86 上交叉构建 Orin 镜像 ----
if [[ $WITH_ARM64 == 1 ]]; then
  log "注册 arm64 binfmt 模拟器"
  docker run --privileged --rm tonistiigi/binfmt --install arm64
fi

log "自检"
docker run --rm hello-world >/dev/null && log "docker 运行正常"
if [[ "$WITH_NVIDIA" == 1 ]]; then
  docker run --rm --gpus all ubuntu:22.04 nvidia-smi -L || log "GPU 容器自检失败（不影响算法，只影响 GUI 加速）"
fi
log "完成"
