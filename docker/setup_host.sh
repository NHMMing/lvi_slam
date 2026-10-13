#!/usr/bin/env bash
# 宿主机一次性准备：安装 Docker Engine（+ buildx/compose 插件），可选 NVIDIA 容器运行时、
# arm64 模拟器。新机器只需要跑这一次，之后所有依赖都在镜像里。
#
# 用法（需要 root；-E 保留当前 shell 的代理变量，没有代理时可省略）:
#   sudo -E docker/setup_host.sh                # Docker + （检测到 NVIDIA 显卡时）nvidia-container-toolkit
#   sudo -E docker/setup_host.sh --arm64        # 另外注册 arm64 模拟器，用于本机交叉构建 Orin 镜像
#   sudo -E docker/setup_host.sh --no-nvidia    # 跳过 NVIDIA 运行时
#
# Docker 软件源（--mirror）：
#   auto（默认）  依次探测：系统 apt 正在用的镜像站 → tuna → aliyun → ustc，用第一个可达的，
#                 直连不走代理；都不可达时退回官方源（走代理）
#   official      官方 download.docker.com（有代理时走代理）
#   <URL>         指定 docker-ce 镜像根地址，如 https://mirrors.tuna.tsinghua.edu.cn/docker-ce
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
MIRROR="${DOCKER_MIRROR:-auto}"
while [[ $# -gt 0 ]]; do
  case "$1" in
    --no-nvidia) WITH_NVIDIA=0 ;;
    --nvidia)    WITH_NVIDIA=1 ;;
    --arm64)     WITH_ARM64=1 ;;
    --mirror)    MIRROR="${2:?--mirror 需要 auto | official | <URL>}"; shift ;;
    --mirror=*)  MIRROR="${1#--mirror=}" ;;
    -h|--help)   sed -n '2,23p' "$0" | sed 's/^# \{0,1\}//'; exit 0 ;;
    *) echo "未知参数: $1" >&2; exit 1 ;;
  esac
  shift
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
[[ -n "$PROXY" ]] && log "检测到代理 $PROXY（apt 只对境外源使用；同时写入 dockerd）"

# apt 一律直连，只有境外的 Docker / NVIDIA 源按主机名单独走代理。
# 原因：sudo -E 会把 http(s)_proxy 带给 apt，而 apt 不认 no_proxy 里 *.cn 这类写法，
# 国内镜像（bfsu/tuna/aliyun...）经代理访问常被拒（403），整个 apt-get update 就失败了。
APT_PROXY_HOSTS=(download.docker.com nvidia.github.io)
apt_get() {
  local opts=() h
  if [[ -n "$PROXY" ]]; then
    for h in "${APT_PROXY_HOSTS[@]}"; do
      opts+=(-o "Acquire::http::Proxy::$h=$PROXY" -o "Acquire::https::Proxy::$h=$PROXY")
    done
  fi
  env -u http_proxy -u https_proxy -u all_proxy -u HTTP_PROXY -u HTTPS_PROXY -u ALL_PROXY \
    apt-get "${opts[@]}" "$@"
}
# 只刷新本脚本添加的那一个源：系统里其他第三方源（VS Code、Chrome、PPA...）
# 连不上或密钥过期都与安装 Docker 无关，不能让它们把整个安装卡住。
apt_update_list() {   # apt_update_list <sources.list.d 下的文件名>
  apt_get update -o Dir::Etc::sourcelist="sources.list.d/$1" \
                 -o Dir::Etc::sourceparts="-" -o APT::Get::List-Cleanup="0" || {
    echo -e "\033[1;31m[setup]\033[0m 刷新 $1 失败，见上面的报错" >&2
    exit 1
  }
}
apt_install() {
  apt_get install -y "$@" || {
    echo -e "\033[1;31m[setup]\033[0m 安装失败。若提示找不到依赖包，先执行 'sudo apt-get update'" \
            "（报错的第三方源可以忽略）后重跑本脚本" >&2
    exit 1
  }
}

# ---- 选择 Docker 软件源 ----
OFFICIAL=https://download.docker.com
direct_ok() {   # 直连（不走 shell 代理）能拿到 <根>/linux/ubuntu/gpg
  env -u http_proxy -u https_proxy -u all_proxy -u HTTP_PROXY -u HTTPS_PROXY -u ALL_PROXY \
    curl -fsS -m 8 -o /dev/null "$1/linux/ubuntu/gpg" 2>/dev/null
}
case "$MIRROR" in
  official) DOCKER_REPO="$OFFICIAL" ;;
  auto)
    DOCKER_REPO=""
    sys_host=$(grep -rhoE '^deb(\s+\[[^]]*\])?\s+https?://[^/ ]+' /etc/apt/sources.list 2>/dev/null |
               grep -oE 'https?://[^/ ]+$' | grep -v 'archive.ubuntu.com\|security.ubuntu.com' | head -1 || true)
    for base in ${sys_host:+"$sys_host/docker-ce"} https://mirrors.tuna.tsinghua.edu.cn/docker-ce \
                https://mirrors.aliyun.com/docker-ce https://mirrors.ustc.edu.cn/docker-ce; do
      if direct_ok "$base"; then DOCKER_REPO="$base"; break; fi
      log "镜像不可用：$base"
    done
    if [[ -z "$DOCKER_REPO" ]]; then
      log "国内镜像都不可达，改用官方源"
      DOCKER_REPO="$OFFICIAL"
    fi ;;
  *) DOCKER_REPO="${MIRROR%/}" ;;
esac
# 镜像站直连；只有官方源才按主机名走代理
if [[ "$DOCKER_REPO" != "$OFFICIAL" ]]; then
  APT_PROXY_HOSTS=(nvidia.github.io)
  CURL_DOCKER=(env -u http_proxy -u https_proxy -u all_proxy -u HTTP_PROXY -u HTTPS_PROXY -u ALL_PROXY curl)
else
  CURL_DOCKER=(curl)
fi
log "Docker 软件源：$DOCKER_REPO"

# ---- 1. Docker Engine ----
. /etc/os-release
if ! command -v docker >/dev/null || ! docker buildx version >/dev/null 2>&1; then
  log "安装 Docker Engine（$VERSION_CODENAME）"
  missing=()
  for p in ca-certificates curl gnupg; do
    dpkg-query -W -f='${Status}' "$p" 2>/dev/null | grep -q "ok installed" || missing+=("$p")
  done
  [[ ${#missing[@]} -eq 0 ]] || apt_install "${missing[@]}"
  install -m 0755 -d /etc/apt/keyrings
  "${CURL_DOCKER[@]}" -fsSL "$DOCKER_REPO/linux/ubuntu/gpg" -o /etc/apt/keyrings/docker.asc
  chmod a+r /etc/apt/keyrings/docker.asc
  echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.asc] \
$DOCKER_REPO/linux/ubuntu $VERSION_CODENAME stable" \
    > /etc/apt/sources.list.d/docker.list
  apt_update_list docker.list
  apt_install docker-ce docker-ce-cli containerd.io \
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
  apt_update_list nvidia-container-toolkit.list
  apt_install nvidia-container-toolkit
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
