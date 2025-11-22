/* Copyright (c) 2021 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import java.util.List;

/*
 * This file contains an example of a Linear "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * Holonomic drives provide the ability for the robot to move in three axes (directions) simultaneously.
 * 1) Axial:    Driving forward and backward               Left-joystick Forward/Backward
 * 2) Lateral:  Strafing right and left                     Left-joystick Right and Left
 * 3) Yaw:      Rotating Clockwise and counter clockwise    Right-joystick Right and Left
 */

@TeleOp(name="AprilTagDT2", group="Linear OpMode")

public class AprilTagDT extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime launchTimer = new ElapsedTime();
    private ElapsedTime intakeJitterTimer = new ElapsedTime();
    private static final boolean USE_WEBCAM = true;
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;
    private DcMotor frontLeftDrive = null;
    private DcMotor backLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor backRightDrive = null;
    private DcMotor intakeMotor = null;
    private DcMotor launchMotor = null;
    private Servo Trigger = null;
    private Servo Roller1 = null;
    private Servo Roller2 = null;
    GoBildaPinpointDriver pinpoint;

    // Launch sequence states
    private enum LaunchState {
        IDLE,
        SPINNING_UP,
        FIRING,
        RESETTING
    }
    private LaunchState launchState = LaunchState.IDLE;

    // Intake jitter states
    private enum IntakeJitterState {
        IDLE,
        REVERSE,
        FORWARD
    }
    private IntakeJitterState intakeJitterState = IntakeJitterState.IDLE;

    // Servo positions - adjust these based on your robot
    private final double TRIGGER_START_POS = 0;
    private final double TRIGGER_FIRE_POS = 300;  // Adjust as needed
    private final double ROLLER_STOP = 300;  // For continuous rotation servos
    private final double ROLLER_FORWARD = 0;  // Adjust speed as needed (0.5-1.0)

    // Timing constants (in seconds)
    private final double SPINUP_TIME_SHORT =1.0;  // Left trigger - quick shot
    private final double SPINUP_TIME_LONG =1.9;   // Right trigger - power shot
    private final double FIRE_TIME = 0.3;          // Time trigger is in fire position
    private final double RESET_TIME = 0.3;         // Time for trigger to reset
    private double currentSpinupTime = SPINUP_TIME_SHORT; // Track which shot type we're using

    // Intake jitter timing
    private final double JITTER_REVERSE_TIME = 0.05;
    private final double JITTER_FORWARD_TIME = 0.10;

    // AprilTag lock-on constants
    private final double TAG_LOCK_KP = 0.02;        // Proportional gain for rotation
    private final double TAG_LOCK_TOLERANCE = 2.0;  // Degrees - how close is "locked on"
    private final double TAG_LOCK_MIN_POWER = 0.05; // Minimum rotation power
    private final double TAG_LOCK_MAX_POWER = 0.4;  // Maximum rotation power

    private double tagX = 0.0;
    private double tagY = 0.0;
    private double tagZ = 0.0;

    private double tagPitch = 0.0;
    private double tagRoll = 0.0;
    private double tagYaw = 0.0;

    private double tagRange = 0.0;
    private double tagBearing = 0.0;
    private double tagElevation = 0.0;

    private int tagId = -1;        // ID of the tag being tracked
    private boolean tagVisible = false; // true if a tag was seen this frame
    private boolean isLockedOn = false; // true if aligned within tolerance

     // Limelight + IMU utility class
    private SharpFaceLimelight3A limelightIMU = null;

    @Override
    public void runOpMode() {
        initAprilTag();

        // Initialize Limelight + IMU utility
        try {
            limelightIMU = new SharpFaceLimelight3A();
            limelightIMU.init(hardwareMap);
            telemetry.addData("Limelight/IMU", "Initialized");
        } catch (Exception e) {
            telemetry.addData("Limelight/IMU", "Not configured - skipping");
            limelightIMU = null;
        }

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        frontLeftDrive = hardwareMap.get(DcMotor.class, "front_left_drive");
        backLeftDrive = hardwareMap.get(DcMotor.class, "back_left_drive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "front_right_drive");
        backRightDrive = hardwareMap.get(DcMotor.class, "back_right_drive");
        intakeMotor = hardwareMap.get(DcMotor.class, "intake_motor");
        launchMotor = hardwareMap.get(DcMotor.class, "launch_motor");
        Trigger = hardwareMap.get(Servo.class, "Trigger");
        Roller1 = hardwareMap.get(Servo.class, "Roller1");
        Roller2 = hardwareMap.get(Servo.class, "Roller2");
        //pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        // ########################################################################################
        // !!!            IMPORTANT Drive Information. Test your motor directions.            !!!!!
        // ########################################################################################
        frontLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        backLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.REVERSE);
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        launchMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        Trigger.setDirection(Servo.Direction.FORWARD);
        Roller1.setDirection(Servo.Direction.FORWARD);
        Roller2.setDirection(Servo.Direction.REVERSE);  // Reverse one so both spin same direction

        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Initialized");
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData("Controls", "Left Trigger = AprilTag Lock-On");
        telemetry.update();

        // Set initial positions
        Trigger.setPosition(TRIGGER_START_POS);
        Roller1.setPosition(ROLLER_STOP);
        Roller2.setPosition(ROLLER_STOP);

        telemetry.addData("Servo", "Starting at 15°");
        telemetry.update();
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            //updates the telemetry for the camera
            telemetryAprilTag();

            // Update Limelight + IMU data
            if (limelightIMU != null) {
                limelightIMU.update();
            }

            double max;

            // ===== GAMEPAD 1 (DRIVER) CONTROLS =====
            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial   = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double lateral =  gamepad1.left_stick_x;
            double yaw;

            // ===== APRILTAG LOCK-ON FEATURE =====
            // Left trigger activates AprilTag lock-on mode
            if (gamepad1.left_trigger > 0.1 && tagVisible) {
                // Use proportional control to align to the tag
                // tagBearing tells us the horizontal angle to the tag
                // Negative bearing = tag is to the left, positive = tag is to the right

                double error = -tagBearing;  // Error in degrees

                // Check if we're locked on (within tolerance)
                if (Math.abs(error) < TAG_LOCK_TOLERANCE) {
                    isLockedOn = true;
                    yaw = 0.0;  // No rotation needed, we're aligned!
                } else {
                    isLockedOn = false;
                    // Calculate proportional control
                    yaw = error * TAG_LOCK_KP;

                    // Apply minimum power to overcome friction
                    if (Math.abs(yaw) > 0 && Math.abs(yaw) < TAG_LOCK_MIN_POWER) {
                        yaw = Math.copySign(TAG_LOCK_MIN_POWER, yaw);
                    }

                    // Clamp to max power
                    yaw = Range.clip(yaw, -TAG_LOCK_MAX_POWER, TAG_LOCK_MAX_POWER);
                }

                telemetry.addData("🎯 TAG LOCK", "ACTIVE");
                telemetry.addData("Bearing Error", "%.1f°", error);
                telemetry.addData("Lock Status", isLockedOn ? "🟢 LOCKED ON" : "🟡 ALIGNING...");
                telemetry.addData("Rotation Power", "%.2f", yaw);
            } else {
                // Normal manual control
                yaw = -gamepad1.right_stick_x;
                isLockedOn = false;

                if (gamepad1.left_trigger > 0.1 && !tagVisible) {
                    telemetry.addData("🎯 TAG LOCK", "⚠️ NO TAG VISIBLE");
                }
            }

            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            double frontLeftPower  = (axial + lateral + yaw) * 1.25;
            double frontRightPower = (axial - lateral - yaw) * 0.75;
            double backLeftPower   = (axial - lateral + yaw) * 1.25;
            double backRightPower  = (axial + lateral - yaw) * 0.75;

            // Normalize the values so no wheel power exceeds 100%
            max = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
            max = Math.max(max, Math.abs(backLeftPower));
            max = Math.max(max, Math.abs(backRightPower));

            if (max > 1.0) {
                frontLeftPower  /= max;
                frontRightPower /= max;
                backLeftPower   /= max;
                backRightPower  /= max;
            }

            // Send calculated power to wheels - ALWAYS UPDATE, NEVER BLOCK
            frontLeftDrive.setPower(frontLeftPower);
            frontRightDrive.setPower(frontRightPower);
            backLeftDrive.setPower(backLeftPower);
            backRightDrive.setPower(backRightPower);

            // Y button: Manual roller control for testing
            if (gamepad1.y) {
                Roller1.setPosition(ROLLER_FORWARD);
                Roller2.setPosition(ROLLER_FORWARD);
            }

            if (limelightIMU != null) {
                // Right trigger resets IMU yaw
                if (gamepad1.right_trigger > 0.5) {
                    limelightIMU.resetIMUYaw();
                }
            }

            // ===== GAMEPAD 2 (OPERATOR) CONTROLS =====

            // ===== INTAKE CONTROL =====
            // Handle intake jitter state machine first (X button)
            switch (intakeJitterState) {
                case IDLE:
                    if (gamepad2.x) {
                        intakeJitterState = IntakeJitterState.REVERSE;
                        intakeJitterTimer.reset();
                    }
                    break;

                case REVERSE:
                    if (intakeJitterTimer.seconds() >= JITTER_REVERSE_TIME) {
                        intakeJitterState = IntakeJitterState.FORWARD;
                        intakeJitterTimer.reset();
                    }
                    break;

                case FORWARD:
                    if (intakeJitterTimer.seconds() >= JITTER_FORWARD_TIME) {
                        intakeJitterState = IntakeJitterState.IDLE;
                    }
                    break;
            }

            // Now handle all intake controls based on button priority
            if (intakeJitterState == IntakeJitterState.REVERSE) {
                // Jitter mode - reverse phase
                intakeMotor.setPower(-1.0);
                Roller1.setPosition(ROLLER_STOP);
                Roller2.setPosition(ROLLER_STOP);
            } else if (intakeJitterState == IntakeJitterState.FORWARD) {
                // Jitter mode - forward phase
                intakeMotor.setPower(1.0);
                Roller1.setPosition(ROLLER_STOP);
                Roller2.setPosition(ROLLER_STOP);
            } else if (gamepad2.a) {
                // A button: Run intake forward with rollers
                intakeMotor.setPower(1.0);
                Roller1.setPosition(ROLLER_FORWARD);
                Roller2.setPosition(ROLLER_FORWARD);
            } else if (gamepad1.y) {
                // Y button: Reverse intake (no rollers)
                intakeMotor.setPower(-1.0);
                Roller1.setPosition(ROLLER_STOP);
                Roller2.setPosition(ROLLER_STOP);
            } else if (gamepad2.b) {
                // B button: Slow intake
                intakeMotor.setPower(0.5);
                Roller1.setPosition(ROLLER_STOP);
                Roller2.setPosition(ROLLER_STOP);
            } else {
                // No buttons pressed: Stop everything
                intakeMotor.setPower(0.0);
                Roller1.setPosition(ROLLER_STOP);
                Roller2.setPosition(ROLLER_STOP);
            }

            // ===== LAUNCH SEQUENCE CONTROL =====
            // State machine for launch sequence
            switch (launchState) {
                case IDLE:
                    // Left trigger = quick shot (1.0s spinup)
                    if (gamepad2.left_trigger > 0.1) {
                        launchState = LaunchState.SPINNING_UP;
                        launchTimer.reset();
                        launchMotor.setPower(1.0);
                        currentSpinupTime = SPINUP_TIME_SHORT;
                        telemetry.addData("🚀 Launch", "Quick Shot - Spinning up...");
                    }
                    // Right trigger = power shot (2.2s spinup)
                    else if (gamepad2.right_trigger > 0.1) {
                        launchState = LaunchState.SPINNING_UP;
                        launchTimer.reset();
                        launchMotor.setPower(1.0);
                        currentSpinupTime = SPINUP_TIME_LONG;
                        telemetry.addData("🚀 Launch", "Power Shot - Spinning up...");
                    }
                    break;

                case SPINNING_UP:
                    // Wait for motor to spin up (uses currentSpinupTime)
                    if (launchTimer.seconds() >= currentSpinupTime) {
                        launchState = LaunchState.FIRING;
                        launchTimer.reset();
                        Trigger.setPosition(TRIGGER_FIRE_POS);
                        telemetry.addData("🚀 Launch", "FIRING!");
                    } else {
                        telemetry.addData("🚀 Launch", "Spinning... %.1fs", launchTimer.seconds());
                    }
                    break;

                case FIRING:
                    // Wait for ball to launch
                    if (launchTimer.seconds() >= FIRE_TIME) {
                        launchState = LaunchState.RESETTING;
                        launchTimer.reset();
                        Trigger.setPosition(TRIGGER_START_POS);
                        launchMotor.setPower(0.0);
                        telemetry.addData("🚀 Launch", "Resetting...");
                    }
                    break;

                case RESETTING:
                    // Wait for trigger to reset, then return to idle
                    if (launchTimer.seconds() >= RESET_TIME) {
                        launchState = LaunchState.IDLE;
                        telemetry.addData("🚀 Launch", "Ready");
                    }
                    break;
            }

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", frontLeftPower, frontRightPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", backLeftPower, backRightPower);
            telemetry.addData("Launch State", launchState);
            telemetry.addData("Intake Jitter", intakeJitterState);
            telemetry.addLine("\n--- GAMEPAD 1 (DRIVER) ---");
            telemetry.addData("Left Trigger", "AprilTag Lock-On");
            telemetry.addData("Y Button", "Test Rollers");
            telemetry.addLine("\n--- GAMEPAD 2 (OPERATOR) ---");
            telemetry.addData("Left Trigger", "Quick Shot (1.0s)");
            telemetry.addData("Right Trigger", "Power Shot (2.2s)");
            telemetry.addData("A Button", "Intake + Rollers");
            telemetry.addData("Y Button", "Reverse Intake");
            telemetry.addData("X Button", "Intake Jitter");
            telemetry.addData("B Button", "Slow Intake");

            // Display Limelight + IMU data if available
            if (limelightIMU != null && limelightIMU.isInitialized()) {
                telemetry.addLine("\n━━━ LIMELIGHT + IMU ━━━");
                telemetry.addData("Pipeline", limelightIMU.getCurrentPipeline());
                telemetry.addData("Limelight Target", limelightIMU.hasValidTarget() ? "LOCKED" : "No Target");

                if (limelightIMU.hasValidTarget()) {
                    telemetry.addData("Vision Yaw/Pitch/Roll", "%.1f° / %.1f° / %.1f°",
                    limelightIMU.getVisionYaw(),
                    limelightIMU.getVisionPitch(),
                    limelightIMU.getVisionRoll());
                    telemetry.addData("Target TX/TY", "%.1f° / %.1f°",
                    limelightIMU.getTargetX(),
                    limelightIMU.getTargetY());
                }

                telemetry.addData("IMU Yaw/Pitch/Roll", "%.1f° / %.1f° / %.1f°",
                limelightIMU.getIMUYaw(),
                limelightIMU.getIMUPitch(),
                limelightIMU.getIMURoll());

                telemetry.addData("D-Pad", "Switch Pipelines (0-3)");
                telemetry.addData("Right Trigger", "Reset IMU Yaw");
            }

            telemetry.update();
        }

       // Cleanup
        if (limelightIMU != null) {
            limelightIMU.stop();
        }
    }

    private void initAprilTag() {

        // Create the AprilTag processor.
        aprilTag = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagOutline(true)
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .build();

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        // Enable the RC preview (LiveView).
        builder.enableLiveView(true);

        // Set and enable the processor.
        builder.addProcessor(aprilTag);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

    }   // end method initAprilTag()


    /**
     * Add telemetry about AprilTag detections.
     */
    private void telemetryAprilTag() {

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        // Step through the list of detections and display info for each one.
        if (currentDetections.size() > 0){
            AprilTagDetection detection = currentDetections.get(0);
            if (detection.metadata != null) {
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
                tagX = detection.ftcPose.x;
                tagY = detection.ftcPose.y;
                tagZ = detection.ftcPose.z;

                tagPitch = detection.ftcPose.pitch;
                tagRoll = detection.ftcPose.roll;
                tagYaw = detection.ftcPose.yaw;

                tagRange = detection.ftcPose.range;
                tagBearing = detection.ftcPose.bearing;
                tagElevation = detection.ftcPose.elevation;

                tagId = detection.id;
                tagVisible = true;

            }
            else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }
        else{
            tagVisible = false;
            tagId = -1;
        }

        // Add "key" information to telemetry
        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
        telemetry.addLine("RBE = Range, Bearing & Elevation");

    }   // end method telemetryAprilTag()

    public void configurePinpoint(){
        pinpoint.setOffsets(-84.0, -168.0, DistanceUnit.MM);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD);
        pinpoint.resetPosAndIMU();
    }

    // Helper to convert degrees (0–300 typical) to normalized 0.0–1.0 servo position
    private static double degToPos(double degrees) {
        // goBILDA servos typically rotate ~300 degrees over full 0–1 range
        return Range.clip(degrees / 300.0, 0.0, 1.0);
    }

}
