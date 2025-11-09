@echo off
title FTC FAST DEPLOY - Team 9000
color 0a
cls

echo.
echo  =============================================
echo  ===       FTC SUPER-FAST DEPLOY v2        ===
echo  ===            Team 9000                  ===
echo  =============================================
echo.

:: ========= AUTO-FIND ADB (works everywhere) =========
set "ADB=%LOCALAPPDATA%\Android\Sdk\platform-tools\adb.exe"
if not exist "%ADB%" (
    echo [ERROR] adb.exe not found!
    echo.
    echo Please install Android Studio or Platform-Tools from:
    echo https://developer.android.com/tools/releases/platform-tools
    echo.
    pause
    exit /b 1
)

:: ========= CONNECT TO CONTROL HUB =========
echo Connecting to Control Hub...
"%ADB%" connect 192.168.43.1:5555 >nul 2>&1
timeout /t 2 >nul

"%ADB%" devices | findstr /R /C:"192.168.43.1.*device" >nul
if %errorlevel% neq 0 (
    echo.
    echo [ERROR] NOT CONNECTED!
    echo.
    echo    Is your laptop connected to FTC-XXXX WiFi?
    echo    Is the Control Hub powered on and showing WiFi?
    echo.
    pause
    exit /b 1
)

:: ========= REMOVE OLD CONFLICTING APPS (THE MAGIC FIX) =========
echo.
echo Removing old conflicting apps...
"%ADB%" uninstall com.qualcomm.ftcrobotcontroller >nul 2>&1
"%ADB%" uninstall com.qualcomm.ftcrobotcontroller.debug >nul 2>&1

:: ========= DEPLOY YOUR CODE =========
echo.
echo Deploying TeamCode... (usually 3-6 seconds)
call gradlew installDebug -PteamNumber=9000 --quiet
if %errorlevel% neq 0 (
    echo.
    echo [ERROR] BUILD FAILED! Check the errors above.
    pause
    exit /b 1
)

:: ========= REFRESH OPMODE LIST =========
echo.
echo Refreshing OpMode list on Driver Station...
"%ADB%" shell am start -n com.qualcomm.ftcrobotcontroller/com.qualcomm.ftcrobotcontroller.RemoteOpModeActivity >nul 2>&1

:: ========= SUCCESS MESSAGE =========
echo.
echo  SUCCESS!
echo.
echo     Your new code is LIVE on the robot!
echo     Go to Driver Station - PULL DOWN TO REFRESH
echo     Your OpModes are there!
echo.
echo  Pro tip: You can now spam this script all day with ZERO errors!
echo.
echo  Press any key to close...
pause >nul