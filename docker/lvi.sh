#!/usr/bin/env bash
# lvi_slam 容器化一键工具。宿主机只需要 Docker（docker/setup_host.sh），其余全在镜像里。
#
# 用法: docker/lvi.sh <命令> [变体] [参数...]
#   变体: noetic（默认）| humble | noros，也可用环境变量 LVI_DISTRO 设定默认值
#
# 镜像
#   pull   [变体]            从 GHCR 拉取预构建的依赖镜像（新机器最快的方式）
#   image  [变体] [选项]     本地构建镜像
#            --target deps|app   默认 deps（开发用）；app = 带编译产物的部署镜像
#            --platform P        如 linux/arm64（Orin），需先 setup_host.sh --arm64
#            --push              构建后推送到 GHCR（需 docker login ghcr.io）
#
# 开发（源码挂载进容器，编译产物放在 ~/.cache/lvi_slam/<变体>，不污染仓库）
#   build  [变体] [--debug] [--no-test]   在容器里增量编译 + 单元测试
#   shell  [变体]                         进入交互式容器
#   run    [变体] -- <命令...>            在容器里执行任意命令，如
#            docker/lvi.sh run noetic -- lvi_offline --config /lvi_slam/config/livox_avia.yaml \
#                --data /data/HKisland01.bag --out-dir /results/hk01
#   eval   [变体] <lvi_eval 参数...>      等价于 run -- lvi_eval ...
#
# 目录映射（环境变量可改）
#   仓库                       → /lvi_slam
#   LVI_DATA    默认 ~/datasets → /data
#   LVI_RESULTS 默认 <仓库>/results → /results
#
# 其他环境变量
#   LVI_REGISTRY  镜像仓库前缀，默认 ghcr.io/nhmming
#   LVI_IMAGE     直接指定镜像（覆盖变体推导）
#   APT_MIRROR    构建时替换 Ubuntu apt 源，如 http://mirrors.tuna.tsinghua.edu.cn
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
REGISTRY="${LVI_REGISTRY:-ghcr.io/nhmming}"
DATA="${LVI_DATA:-$HOME/datasets}"
RESULTS="${LVI_RESULTS:-$REPO/results}"
CACHE_ROOT="${XDG_CACHE_HOME:-$HOME/.cache}/lvi_slam"

die()  { echo -e "\033[1;31m[lvi]\033[0m $*" >&2; exit 1; }
log()  { echo -e "\033[1;32m[lvi]\033[0m $*" >&2; }
usage() { sed -n '2,35p' "$0" | sed 's/^# \{0,1\}//'; }

command -v docker >/dev/null || die "未安装 Docker，先运行: sudo -E docker/setup_host.sh"

# ---- 解析变体 ----
distro_of() {   # 变体名 → 镜像里的 ROS_DISTRO
  case "$1" in
    noetic|ros1) echo noetic ;;
    humble|ros2) echo humble ;;
    noros|none)  echo none ;;
    *) return 1 ;;
  esac
}
variant_of() { [[ "$1" == none ]] && echo noros || echo "$1"; }

CMD="${1:-}"; shift || true
[[ -z "$CMD" || "$CMD" == -h || "$CMD" == --help ]] && { usage; exit 0; }

DISTRO="$(distro_of "${LVI_DISTRO:-noetic}")" || die "LVI_DISTRO 无效: $LVI_DISTRO"
if [[ $# -gt 0 ]] && d="$(distro_of "$1" 2>/dev/null)"; then DISTRO="$d"; shift; fi
VARIANT="$(variant_of "$DISTRO")"

image_name() {   # image_name <target>
  [[ -n "${LVI_IMAGE:-}" ]] && { echo "$LVI_IMAGE"; return; }
  [[ "$1" == app ]] && echo "$REGISTRY/lvi_slam:$VARIANT" || echo "$REGISTRY/lvi_slam:$VARIANT-deps"
}

# 宿主机的代理（127.0.0.1:xxxx）在 host 网络下容器里也能用，原样传进去
proxy_env() {
  local v
  for v in http_proxy https_proxy all_proxy no_proxy HTTP_PROXY HTTPS_PROXY ALL_PROXY NO_PROXY; do
    [[ -n "${!v:-}" ]] && printf -- '-e\n%s=%s\n' "$v" "${!v}"
  done
  return 0
}

# ---- 构建镜像 ----
cmd_image() {
  local target=deps platform="" push=0
  while [[ $# -gt 0 ]]; do
    case "$1" in
      --target)   target="$2"; shift ;;
      --platform) platform="$2"; shift ;;
      --push)     push=1 ;;
      *) die "image: 未知参数 $1" ;;
    esac
    shift
  done
  [[ "$target" == deps || "$target" == app ]] || die "--target 只能是 deps 或 app"

  local tag rev args=()
  tag="$(image_name "$target")"
  rev="$(git -C "$REPO" describe --always --dirty --abbrev=12 2>/dev/null || echo unknown)"
  args=(--file "$REPO/docker/Dockerfile" --target "$target"
        --build-arg ROS_DISTRO="$DISTRO" --build-arg LVI_GIT_REV="$rev"
        --build-arg APT_MIRROR="${APT_MIRROR:-}"
        --network host --tag "$tag")
  # 代理变量是 Docker 预定义的构建参数，不会写进镜像层
  local v
  for v in http_proxy https_proxy no_proxy HTTP_PROXY HTTPS_PROXY NO_PROXY; do
    [[ -n "${!v:-}" ]] && args+=(--build-arg "$v=${!v}")
  done
  if [[ -n "$platform" ]]; then
    args+=(--platform "$platform")
    # 跨架构镜像不能 --load 到本地多架构以外的 docker 存储时，用 push
    [[ $push == 1 ]] && args+=(--push) || args+=(--load)
  else
    args+=(--load)
  fi
  log "构建 $tag（ROS_DISTRO=$DISTRO target=$target ${platform:+platform=$platform}）"
  docker buildx build "${args[@]}" "$REPO"
  if [[ $push == 1 && -z "$platform" ]]; then docker push "$tag"; fi
  log "完成: $tag"
}

ensure_image() {   # 本地没有 deps 镜像时先尝试 pull，失败再本地构建
  local tag; tag="$(image_name deps)"
  docker image inspect "$tag" >/dev/null 2>&1 && return
  log "本地没有 $tag，尝试从仓库拉取..."
  docker pull "$tag" 2>/dev/null || { log "拉取失败，改为本地构建（首次约 20-40 分钟，主要是编译 GTSAM）"; cmd_image; }
}

# ---- 运行容器 ----
run_container() {   # run_container <是否交互 0|1> <命令...>
  local interactive="$1"; shift
  ensure_image
  local ws="$CACHE_ROOT/$VARIANT"
  mkdir -p "$ws" "$RESULTS" "$DATA" 2>/dev/null || true

  local args=(--rm --network host --ipc host
    --user "$(id -u):$(id -g)" -e HOME=/tmp/home
    -v "$REPO:/lvi_slam" -v "$ws:/ws" -v "$DATA:/data" -v "$RESULTS:/results"
    -w /results)
  mapfile -t penv < <(proxy_env)
  args+=("${penv[@]}")
  [[ $interactive == 1 ]] && args+=(-it)
  [[ -t 1 && $interactive == 0 ]] && args+=(-t)

  # GUI（RViz 等）：有 X 显示时转发；有 NVIDIA 运行时就用 GPU 渲染
  if [[ -n "${DISPLAY:-}" && -d /tmp/.X11-unix ]]; then
    args+=(-e DISPLAY="$DISPLAY" -v /tmp/.X11-unix:/tmp/.X11-unix:ro)
    [[ -f "${XAUTHORITY:-$HOME/.Xauthority}" ]] &&
      args+=(-e XAUTHORITY=/tmp/.Xauthority -v "${XAUTHORITY:-$HOME/.Xauthority}:/tmp/.Xauthority:ro")
  fi
  if docker info 2>/dev/null | grep -q 'Runtimes:.*nvidia'; then
    args+=(--gpus all -e NVIDIA_DRIVER_CAPABILITIES=all)
  fi

  docker run "${args[@]}" "$(image_name deps)" "$@"
}

cmd_build() {
  local bt=Release tests=1
  while [[ $# -gt 0 ]]; do
    case "$1" in
      --debug)   bt=Debug ;;
      --no-test) tests=0 ;;
      *) die "build: 未知参数 $1" ;;
    esac
    shift
  done
  run_container 0 env BUILD_TYPE="$bt" RUN_TESTS="$tests" /lvi_slam/docker/ws_build.sh
}

case "$CMD" in
  pull)  docker pull "$(image_name deps)" ;;
  image) cmd_image "$@" ;;
  build) cmd_build "$@" ;;
  shell) run_container 1 bash ;;
  run)   [[ "${1:-}" == -- ]] && shift; [[ $# -gt 0 ]] || die "run 需要命令"; run_container 0 "$@" ;;
  eval)  run_container 0 lvi_eval "$@" ;;
  *) usage; exit 1 ;;
esac
