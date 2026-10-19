#!/bin/bash
set -e

cd "$(dirname "$0")/.."

echo "[INFO] Configuring..."
cmake -B build -DCMAKE_BUILD_TYPE=Release

echo "[INFO] Building..."
cmake --build build -j$(nproc)

echo "[INFO] Build finished. Run: ./build/armor_detector config/default_params.yaml data/sample.jpg"
