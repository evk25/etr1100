#!/usr/bin/env bash
set -euo pipefail

FIRMWARE_BIN="ESP32_GENERIC-20231227-v1.22.0.bin"
FIRMWARE_DIR="bin"
FIRMWARE_PATH="${FIRMWARE_DIR}/${FIRMWARE_BIN}"
FIRMWARE_URL="https://micropython.org/resources/firmware/${FIRMWARE_BIN}"

echo "==> Setting up Python virtual environment with uv..."
if [ ! -d ".venv" ]; then
    uv venv
fi
uv pip install -r requirements.txt

echo "==> Checking MicroPython ESP32 firmware (${FIRMWARE_PATH})..."
mkdir -p "${FIRMWARE_DIR}"
if [ ! -f "${FIRMWARE_PATH}" ]; then
    curl -fL -o "${FIRMWARE_PATH}" "${FIRMWARE_URL}"
    echo "Downloaded ${FIRMWARE_PATH}"
else
    echo "${FIRMWARE_PATH} already exists, skipping download."
fi

echo ""
echo "Setup complete!"
echo "Next steps:"
echo "  1. Start Wokwi Simulator in VS Code (F1 -> Wokwi: Start Simulator)"
echo "  2. Upload Python files to simulator:"
echo "       .venv/bin/mpremote connect port:rfc2217://localhost:4000 fs cp main.py : + fs cp -r src :"
echo "  3. Reset ESP32 (Ctrl+D in Serial Monitor) to run main.py"
