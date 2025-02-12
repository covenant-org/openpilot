#!/usr/bin/env bash

export OMP_NUM_THREADS=1
export MKL_NUM_THREADS=1
export NUMEXPR_NUM_THREADS=1
export OPENBLAS_NUM_THREADS=1
export VECLIB_MAXIMUM_THREADS=1

if [ -z "$AGNOS_VERSION" ]; then
  export AGNOS_VERSION="12.3"
fi

export STAGING_ROOT="/data/safe_staging"
export LD_LIBRARY_PATH="/data/MAVSDK/install/lib:/data/zlib-1.3.1/install/lib:$LD_LIBRARY_PATH"
