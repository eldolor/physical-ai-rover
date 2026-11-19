#!/usr/bin/env bash
# Build the ROS image in a cloud session (3.189), where the only egress is an
# HTTPS proxy that re-terminates TLS with its own CA, and packages.ros.org
# (HTTP-only for apt) is unreachable. Not for the desktop or the Jetson: they
# run `docker build -t ... service/slam` and are unaffected.
#
#   bash service/slam/build-cloud.sh [TAG]       # default vision-picar-ros:cloud
#
# What it does: (1) builds a local base from ros:humble-ros-base that trusts
# the proxy CA, sends apt through the proxy over HTTPS and takes the ROS
# packages from the OSUOSL mirror; (2) builds service/slam/Dockerfile on it
# with BASE_IMAGE. The CA is copied into a temporary directory and never
# lands in the repo; it does end up inside the (local-only) image.
# tf2 0.25.24 and the pinned slam_toolbox are still built from source.
set -euo pipefail

TAG="${1:-vision-picar-ros:cloud}"
case "$TAG" in *:latest) echo "refusing :latest (the shared tag)"; exit 2;; esac
: "${HTTPS_PROXY:?HTTPS_PROXY is not set (is this a cloud session?)}"
CA="${PROXY_CA:-/root/.ccr/agent-proxy-ca.crt}"
[ -r "$CA" ] || { echo "no proxy CA at $CA (set PROXY_CA)"; exit 2; }
MIRROR="${ROS_APT_MIRROR:-https://ftp.osuosl.org/pub/ros/packages.ros.org/ros2/ubuntu}"
HERE="$(cd "$(dirname "$0")" && pwd)"
BASE="${TAG%%:*}-base:${TAG##*:}"

TMP="$(mktemp -d)"; trap 'rm -rf "$TMP"' EXIT
cp "$CA" "$TMP/proxy-ca.crt"
cat > "$TMP/Dockerfile" <<DOCKER
FROM ros:humble-ros-base
COPY proxy-ca.crt /usr/local/share/ca-certificates/proxy-ca.crt
RUN update-ca-certificates \\
 && printf 'Acquire::https::Proxy "%s";\\n' "$HTTPS_PROXY" > /etc/apt/apt.conf.d/99proxy \\
 && sed -i -E 's#http://(archive|security)\\.ubuntu\\.com#https://\\1.ubuntu.com#g' /etc/apt/sources.list \\
 && sed -i 's#http://packages.ros.org/ros2/ubuntu#$MIRROR#g' /etc/apt/sources.list.d/*.sources \\
 &&  true
ENV GIT_SSL_CAINFO=/etc/ssl/certs/ca-certificates.crt
DOCKER

# --network host: the proxy listens on the host's 127.0.0.1.
docker build --network host -t "$BASE" "$TMP"
docker build --network host \
    --build-arg "BASE_IMAGE=$BASE" \
    --build-arg "HTTPS_PROXY=$HTTPS_PROXY" --build-arg "https_proxy=$HTTPS_PROXY" \
    --build-arg "GIT_SSL_CAINFO=/etc/ssl/certs/ca-certificates.crt" \
    -t "$TAG" "$HERE"
docker image ls "$TAG"
