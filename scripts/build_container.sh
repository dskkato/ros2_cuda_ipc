#!/usr/bin/env bash
Set -euo pipefail

# Build the development container image used by the project

usage() {
  cat <<'USAGE'
Usage: build_container.sh [options]

Options:
  -t, --tag TAG            Set the image tag (default: ros2-cuda-ipc-dev:latest)
  -r, --ros-distro DISTRO  Override ROS_DISTRO build arg (default: humble)
      --build-arg ARG      Provide an additional docker build argument (repeatable)
      --no-cache           Disable build cache
      --push               Push the image after a successful build
  -h, --help               Show this help message and exit
USAGE
}

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DOCKERFILE_PATH="${SCRIPT_DIR}/Dockerfile"
BUILD_CONTEXT="${SCRIPT_DIR}"
REGISTRY_URL="ghcr.io/dskkato"
DEFAULT_TAG="ros2-cuda-ipc:latest"
DEFAULT_ROS_DISTRO="humble"

TAG="${DEFAULT_TAG}"
ROS_DISTRO="${ROS_DISTRO:-${DEFAULT_ROS_DISTRO}}"
NO_CACHE=false
PUSH_AFTER=false
EXTRA_BUILD_ARGS=()

while [[ $# -gt 0 ]]; do
  case "$1" in
    -t|--tag)
      [[ $# -ge 2 ]] || { echo "Error: missing value for $1" >&2; exit 1; }
      TAG="$2"
      shift 2
      ;;
    -r|--ros-distro)
      [[ $# -ge 2 ]] || { echo "Error: missing value for $1" >&2; exit 1; }
      ROS_DISTRO="$2"
      shift 2
      ;;
    --build-arg)
      [[ $# -ge 2 ]] || { echo "Error: missing value for $1" >&2; exit 1; }
      EXTRA_BUILD_ARGS+=("$2")
      shift 2
      ;;
    --no-cache)
      NO_CACHE=true
      shift
      ;;
    --push)
      PUSH_AFTER=true
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "Error: unknown option $1" >&2
      usage
      exit 1
      ;;
  esac
done

if [[ ! -f "${DOCKERFILE_PATH}" ]]; then
  echo "Error: Dockerfile not found at ${DOCKERFILE_PATH}" >&2
  exit 1
fi

if ! command -v docker >/dev/null 2>&1; then
  echo "Error: docker CLI is not available in PATH" >&2
  exit 1
fi

ARTIFACT="${REGISTRY_URL}/${TAG}"
BUILD_CMD=(docker build "${BUILD_CONTEXT}" --file "${DOCKERFILE_PATH}" --tag "${ARTIFACT}" --build-arg "ROS_DISTRO=${ROS_DISTRO}")

if ${NO_CACHE}; then
  BUILD_CMD+=(--no-cache)
fi

if [[ ${#EXTRA_BUILD_ARGS[@]} -gt 0 ]]; then
  for arg in "${EXTRA_BUILD_ARGS[@]}"; do
    BUILD_CMD+=(--build-arg "$arg")
  done
fi

echo "Building container image ${ARTIFACT} (ROS_DISTRO=${ROS_DISTRO}) using ${DOCKERFILE_PATH}" >&2
"${BUILD_CMD[@]}"

if ${PUSH_AFTER}; then
  echo "Pushing image ${ARTIFACT}" >&2
  docker push "${ARTIFACT}"
fi

