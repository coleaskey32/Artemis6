#!/bin/bash
# Import URDF into Webots

echo "🤖 URDF → Webots Import Helper"
echo "================================"
echo ""
echo "This will convert robot.urdf to a Webots PROTO file."
echo ""

# Check if urdf2webots exists
CONVERTER="/Applications/Webots.app/Contents/lib/controller/urdf2webots"

if [ ! -f "$CONVERTER" ]; then
    echo "❌ Error: urdf2webots not found!"
    echo "   Expected at: $CONVERTER"
    echo ""
    echo "Please use Webots GUI to import:"
    echo "  1. Open Webots"
    echo "  2. File → Import → URDF"
    echo "  3. Select: urdf/robot.urdf"
    exit 1
fi

# Get script directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROBOT_DIR="$(dirname "$SCRIPT_DIR")"

echo "📁 Working directory: $ROBOT_DIR"
echo ""

# Create protos directory if it doesn't exist
mkdir -p "$ROBOT_DIR/protos"

# Convert URDF
echo "🔄 Converting URDF to Webots PROTO..."
"$CONVERTER" \
  --input="$ROBOT_DIR/urdf/robot.urdf" \
  --output="$ROBOT_DIR/protos/MobileManipulator.proto" \
  --robot-name=mobile_manipulator \
  --normal \
  --box-collision \
  --tool-slot=tool0

if [ $? -eq 0 ]; then
    echo ""
    echo "✅ Conversion successful!"
    echo ""
    echo "📝 Next steps:"
    echo "  1. Open world.wbt in Webots"
    echo "  2. Add this line to the world file:"
    echo ""
    echo "     MobileManipulator {"
    echo "       translation 0 0 0.1"
    echo "       controller \"robot_controller\""
    echo "     }"
    echo ""
    echo "  Or use: File → Add → Import... → protos/MobileManipulator.proto"
else
    echo ""
    echo "❌ Conversion failed!"
    echo "   Please use Webots GUI import instead."
fi

