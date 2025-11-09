#!/bin/bash
echo ""
echo "=== FTC DEPLOY ==="
echo ""

PATH="/Users/omsavani/Library/Android/sdk/platform-tools:$PATH"

# Auto-connect + show connection
adb connect 192.168.43.1:5555 2>/dev/null
adb devices | grep 192.168.43.1 | grep device || {
    echo ""
    echo "NOT CONNECTED!"
    echo "→ Is your laptop on the FTC-XXXX WiFi?"
    echo "→ Is the Control Hub powered on?"
    read -p "Press Enter to exit..."
    exit 1
}

echo "Removing old conflicting apps..."
adb uninstall com.qualcomm.ftcrobotcontroller 2>/dev/null || true
adb uninstall com.qualcomm.ftcrobotcontroller.debug 2>/dev/null || true

echo ""
echo "Deploying your code..."
./gradlew installDebug -PteamNumber=9000 --quiet

echo ""
echo "Refreshing OpModes..."
adb shell am start -n com.qualcomm.ftcrobotcontroller/com.qualcomm.ftcrobotcontroller.RemoteOpModeActivity >/dev/null 2>&1

echo ""
echo "DONE! New OpModes are LIVE!"
echo "Pull down on Driver Station → they’re there!"
echo ""
read -p "Press Enter to close..."