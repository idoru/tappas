#!/usr/bin/env bash

export TAPPAS_WORKSPACE=$(pwd)
export CAMSERV_WORKSPACE=$TAPPAS_WORKSPACE/../camserv

repave() {
  pushd "$TAPPAS_WORKSPACE"
  ./scripts/gstreamer/install_hailo_gstreamer.sh --skip-hailort
  popd

  pushd "$CAMSERV_WORKSPACE"
  make clean
  make all
  sudo make install
  sudo systemctl restart webcam.service
  popd
}

if [ "$1" = "--repave" ]; then
  repave
  exit 0
fi

set -exuo pipefail
find . -type f -name '*.cpp' -or -name '*.hpp' -or -name '*.c' -or -name '*.h' | entr -s "$0 --repave"
