package org.firstinspires.ftc.teamcode.Calibration;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;

@TeleOp(name="CameraCalibration", group="Vision")
public class CameraCalibration extends LinearOpMode {

    private Limelight3A limelight;

    @Override
    public void runOpMode() {
        // Initialize Limelight
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Get the latest result from Limelight
            LLResult result = limelight.getLatestResult();

            if (result != null && result.isValid()) {
                // Check if AprilTag is detected
                if (!result.getFiducialResults().isEmpty()) {

                    // Get the first detected AprilTag
                    LLResultTypes.FiducialResult fiducial = result.getFiducialResults().get(0);

                    // Get the 3D pose of the AprilTag relative to the camera
                    Pose3D tagPose = fiducial.getTargetPoseCameraSpace();

                    if (tagPose != null) {
                        Position position = tagPose.getPosition();

                        // Calculate distance using 3D coordinates (x, y, z)
                        double distance = Math.sqrt(
                                Math.pow(position.x, 2) +
                                        Math.pow(position.y, 2) +
                                        Math.pow(position.z, 2)
                        );

                        // Display results
                        telemetry.addData("AprilTag ID", fiducial.getFiducialId());
                        telemetry.addData("Distance (inches)", "%.2f", distance);
                        telemetry.addData("X Position", "%.2f", position.x);
                        telemetry.addData("Y Position", "%.2f", position.y);
                        telemetry.addData("Z Position", "%.2f", position.z);

                        // For FTC, Z is typically the most important for distance
                        telemetry.addData("Forward Distance (Z)", "%.2f", position.z);

                    } else {
                        telemetry.addData("Status", "AprilTag pose not available");
                    }

                } else {
                    telemetry.addData("Status", "No AprilTags detected");
                }

            } else {
                telemetry.addData("Status", "No valid results from Limelight");
            }

            telemetry.update();
            sleep(50); // Small delay to prevent excessive processing
        }
    }
}
