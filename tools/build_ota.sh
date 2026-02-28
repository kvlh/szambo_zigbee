#!/bin/bash
# Build firmware and generate OTA image for Z2M
#
# Prerequisites:
#   - ESP-IDF environment sourced (. $IDF_PATH/export.sh)
#   - esp-zigbee-sdk cloned (for image_builder_tool.py)
#
# Usage: ./tools/build_ota.sh [version_hex]
#   version_hex: OTA version in hex (default: 0x01000001)

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"
BUILD_DIR="$PROJECT_DIR/build"
OTA_DIR="$PROJECT_DIR/ota_images"

# OTA parameters (must match zigbee_device.h)
MANUFACTURER=0x1001
IMAGE_TYPE=0x1011
VERSION=${1:-0x01000001}

# Colors
GREEN='\033[0;32m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

echo -e "${BLUE}==============================${NC}"
echo -e "${BLUE}  Build Szambo TOF OTA Image  ${NC}"
echo -e "${BLUE}==============================${NC}"
echo ""

# Build firmware
echo -e "${BLUE}[1/3] Building firmware...${NC}"
cd "$PROJECT_DIR"
idf.py build

echo -e "${GREEN}Firmware built${NC}"
echo ""

# Find the app binary
APP_BIN="$BUILD_DIR/szambo_tof_native.bin"
if [ ! -f "$APP_BIN" ]; then
    echo -e "${RED}ERROR: $APP_BIN not found${NC}"
    exit 1
fi

# Create OTA output directory
mkdir -p "$OTA_DIR"

# Generate OTA image
echo -e "${BLUE}[2/3] Generating OTA image...${NC}"

# Try to find image_builder_tool.py
IMAGE_BUILDER=""
for path in \
    "$IDF_PATH/../esp-zigbee-sdk/tools/image_builder_tool.py" \
    "$HOME/esp/esp-zigbee-sdk/tools/image_builder_tool.py" \
    "$(which image_builder_tool.py 2>/dev/null)" \
    ; do
    if [ -f "$path" ]; then
        IMAGE_BUILDER="$path"
        break
    fi
done

OTA_FILE="$OTA_DIR/szambo_tof_native_${VERSION}.ota"

if [ -n "$IMAGE_BUILDER" ]; then
    python3 "$IMAGE_BUILDER" \
        --manufacturer "$MANUFACTURER" \
        --image-type "$IMAGE_TYPE" \
        --file-version "$VERSION" \
        --input "$APP_BIN" \
        --output "$OTA_FILE"
    echo -e "${GREEN}OTA image created: $OTA_FILE${NC}"
else
    echo -e "${YELLOW}WARNING: image_builder_tool.py not found${NC}"
    echo "You can generate the OTA image manually:"
    echo "  python3 image_builder_tool.py \\"
    echo "    --manufacturer $MANUFACTURER \\"
    echo "    --image-type $IMAGE_TYPE \\"
    echo "    --file-version $VERSION \\"
    echo "    --input $APP_BIN \\"
    echo "    --output $OTA_FILE"
    echo ""
    echo "For now, copying raw binary..."
    cp "$APP_BIN" "$OTA_FILE"
fi

# Update ota_index.json
echo -e "${BLUE}[3/3] Updating OTA index...${NC}"
OTA_INDEX="$PROJECT_DIR/z2m/ota_index.json"
cat > "$OTA_INDEX" << EOF
[
  {
    "url": "szambo_tof_native_${VERSION}.ota",
    "force": false,
    "manufacturerCode": $((MANUFACTURER)),
    "imageType": $((IMAGE_TYPE)),
    "fileVersion": $((VERSION))
  }
]
EOF

echo -e "${GREEN}OTA index updated: $OTA_INDEX${NC}"
echo ""
echo -e "${BLUE}==============================${NC}"
echo -e "${GREEN}  Build complete!${NC}"
echo -e "${BLUE}==============================${NC}"
echo ""
echo "To deploy OTA to Z2M:"
echo "  1. Copy $OTA_FILE to Z2M data directory"
echo "  2. Copy $OTA_INDEX to Z2M data directory"
echo "  3. Set ota.zigbee_ota_override_index_location in Z2M config"
echo "  4. Restart Z2M and trigger OTA from device page"
echo ""
