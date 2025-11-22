/*
 * SharpFace Limelight3A
 * 
 * This class provides:
 * - Limelight3A vision tracking with pose estimation
 * - IMU orientation sensing (yaw, pitch, roll)
 * - Pipeline switching for different vision modes
 * - Easy integration into any OpMode
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public class SharpFaceLimelight3A {

    // Hardware objects
    private Limelight3A limelight = null;
    private IMU imu = null

    // Cached sensor data
    private LLResult latestResult = null;
    private LLStatus latestStatus = null;
    private YawPitchRollAngles imuOrientation = null;
    private AngularVelocity imuVelocity = null;

    // Current pipeline
    private int currentPipeline = 0;

    // Initialization flag
    private boolean initialized = false;

    /**
     * Constructor
     */
    public SharpFaceLimelight3A() {
        // Empty constructor
    }

    /**
     * Initialize the Limelight and IMU hardware.
     * This must be called once during OpMode initialization.
     * 
     * @param hardwareMap The hardware map from the OpMode
     */
    public void init(HardwareMap hardwareMap) {
        init(hardwareMap, 
        RevHubOrientationOnRobot.LogoFacingDirection.UP,
        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD);
    }

    /**
     * Initialize the Limelight and IMU hardware with custom hub orientation.
     * 
     * @param hardwareMap The hardware map from the OpMode
     * @param logoDirection Direction the REV Hub logo is facing
     * @param usbDirection Direction the REV Hub USB port is facing
     */
    public void init(HardwareMap hardwareMap, 
    RevHubOrientationOnRobot.LogoFacingDirection logoDirection,
    RevHubOrientationOnRobot.UsbFacingDirection usbDirection) {

        // Initialize Limelight
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();

        // Initialize IMU
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot orientationOnRobot = 
        new RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        initialized = true;
    }

    /**
     * Update sensor data. Call this in your OpMode loop to get fresh data.
     */
    public void update() {
        if (!initialized) {
         throw new IllegalStateException("SharpFaceLimelight3A not initialized! Call init() first.");
        }

        // Update Limelight data
        latestStatus = limelight.getStatus();
        latestResult = limelight.getLatestResult();

        // Update IMU data
        imuOrientation = imu.getRobotYawPitchRollAngles();
        imuVelocity = imu.getRobotAngularVelocity(AngleUnit.DEGREES);
    }

    /**
     * Switch to a different vision pipeline.
     * 
     * @param pipeline Pipeline index (0-3 typically)
     */
    public void switchPipeline(int pipeline) {
        if (!initialized) return;
        currentPipeline = pipeline;
        limelight.pipelineSwitch(pipeline);
    }

    /**
     * Reset the IMU yaw (heading) to zero.
     */
    public void resetIMUYaw() {
        if (!initialized) return;
        imu.resetYaw();
    }

    /**
     * Stop the Limelight. Call this when shutting down the OpMode.
     */
    public void stop() {
        if (limelight != null) {
        limelight.stop();
        }
    }

    // ========== VISION DATA ACCESSORS ==========

    /**
     * Get the latest Limelight result.
     * @return LLResult object, or null if not initialized
     */
    public LLResult getLatestResult() {
        return latestResult;
    }

    /**
     * Get the latest Limelight status.
     * @return LLStatus object, or null if not initialized
     */
    public LLStatus getStatus() {
        return latestStatus;
    }

    /**
     * Check if vision has a valid target.
     * @return true if a valid target is detected
     */
    public boolean hasValidTarget() {
        return latestResult != null && latestResult.isValid();
    }

    /**
     * Get the robot's pose from vision tracking.
     * @return Pose3D object, or null if no valid target
     */
    public Pose3D getVisionPose() {
        if (!hasValidTarget()) return null;
        return latestResult.getBotpose();
    }

    /**
     * Get vision-based yaw angle.
     * @return Yaw in degrees, or 0 if no valid target
    */

    public double getVisionYaw() {
        Pose3D pose = getVisionPose();
        if (pose == null) return 0;
        return pose.getOrientation().getYaw(AngleUnit.DEGREES);
    }

    /**
     * Get vision-based pitch angle.
     * @return Pitch in degrees, or 0 if no valid target
     */
    public double getVisionPitch() {
        Pose3D pose = getVisionPose();
        if (pose == null) return 0;
        return pose.getOrientation().getPitch(AngleUnit.DEGREES);
    }

    /**
     * Get vision-based roll angle.
     * @return Roll in degrees, or 0 if no valid target
     */
    public double getVisionRoll() {
        Pose3D pose = getVisionPose();
        if (pose == null) return 0;
        return pose.getOrientation().getRoll(AngleUnit.DEGREES);
    }

    /**
     * Get horizontal offset to target (TX).
     * @return Horizontal angle in degrees, or 0 if no valid target
     */
    public double getTargetX() {
        if (!hasValidTarget()) return 0;
        return latestResult.getTx();
    }

    /**
     * Get vertical offset to target (TY).
     * @return Vertical angle in degrees, or 0 if no valid target
     */
    public double getTargetY() {
        if (!hasValidTarget()) return 0;
        return latestResult.getTy();
    }

    /**
     * Get vision X position.
     * @return X position in inches, or 0 if no valid target
     */
    public double getVisionX() {
        Pose3D pose = getVisionPose();
        if (pose == null) return 0;
        return pose.getPosition().x;
    }

    /**
     * Get vision Y position.
     * @return Y position in inches, or 0 if no valid target
     */
    public double getVisionY() {
        Pose3D pose = getVisionPose();
        if (pose == null) return 0;
        return pose.getPosition().y;
    }

    /**
     * Get vision Z position.
     * @return Z position in inches, or 0 if no valid target
     */
    public double getVisionZ() {
        Pose3D pose = getVisionPose();
        if (pose == null) return 0;
        return pose.getPosition().z;
    }

    // ========== IMU DATA ACCESSORS ==========

    /**
     * Get IMU yaw (heading) angle.
     * @return Yaw in degrees
     */
    public double getIMUYaw() {
        if (imuOrientation == null) return 0;
        return imuOrientation.getYaw(AngleUnit.DEGREES);
    }

    /**
     * Get IMU pitch angle.
     * @return Pitch in degrees
     */
    public double getIMUPitch() {
        if (imuOrientation == null) return 0;
        return imuOrientation.getPitch(AngleUnit.DEGREES);
    }

    /**
     * Get IMU roll angle.
     * @return Roll in degrees
     */
    public double getIMURoll() {
        if (imuOrientation == null) return 0;
        return imuOrientation.getRoll(AngleUnit.DEGREES);
    }

    /**
     * Get IMU yaw rotation rate.
     * @return Yaw velocity in degrees/second
     */
    public double getIMUYawVelocity() {
        if (imuVelocity == null) return 0;
        return imuVelocity.zRotationRate;
    }

    /**
     * Get IMU pitch rotation rate.
     * @return Pitch velocity in degrees/second
     */
    public double getIMUPitchVelocity() {
        if (imuVelocity == null) return 0;
        return imuVelocity.xRotationRate;
    }

    /**
     * Get IMU roll rotation rate.
     * @return Roll velocity in degrees/second
     */
    public double getIMURollVelocity() {
        if (imuVelocity == null) return 0;
        return imuVelocity.yRotationRate;
    }

    /**
     * Get the full IMU orientation object.
     * @return YawPitchRollAngles object
     */
    public YawPitchRollAngles getIMUOrientation() {
        return imuOrientation;
    }

    /**
     * Get the full IMU velocity object.
     * @return AngularVelocity object
     */
    public AngularVelocity getIMUVelocity() {
        return imuVelocity;
    }

    // ========== STATUS ACCESSORS ==========

    /**
     * Get current pipeline index.
     * @return Pipeline index
     */
    public int getCurrentPipeline() {
        return currentPipeline;
    }

    /**
     * Get Limelight temperature.
     * @return Temperature in Celsius
     */
    public double getTemperature() {
        if (latestStatus == null) return 0;
        return latestStatus.getTemp();
    }

    /**
     * Get Limelight CPU usage.
     * @return CPU usage percentage
     */
    public double getCPUUsage() {
        if (latestStatus == null) return 0;
        return latestStatus.getCpu();
    }

    /**
     * Get Limelight frames per second.
     * @return FPS
     */
    public double getFPS() {
        if (latestStatus == null) return 0;
        return latestStatus.getFps();
    }

    /**
     * Get total vision latency (capture + targeting).
     * @return Latency in milliseconds
     */
    public double getVisionLatency() {
        if (!hasValidTarget()) return 0;
        return latestResult.getCaptureLatency() + latestResult.getTargetingLatency();
    }

    /**
     * Check if the utility class has been initialized.
     * @return true if initialized
     */
    public boolean isInitialized() {
        return initialized;
    }
}