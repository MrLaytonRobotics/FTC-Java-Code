package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.ArrayList;
import java.util.List;

@Config
@TeleOp(name = "Test Vision", group = "Vision")
public class TestVision extends LinearOpMode {

    private OpenCvWebcam webcam;
    private FtcDashboard dashboard;

    // Tunable HSV thresholds for live dashboard adjustment
    public static double hMin = 0, sMin = 0, vMin = 0;
    public static double hMax = 180, sMax = 255, vMax = 255;

    // Outputs for telemetry
    public static double detectedX = -1;
    public static double detectedY = -1;
    public static double detectedArea = 0;

    @Override
    public void runOpMode() {
        // Create webcam WITHOUT DS monitor view (dashboard handles stream)
        webcam = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "Webcam 1"));

        // Set the OpenCV pipeline
        webcam.setPipeline(new BallDetectPipeline());

        // Initialize Dashboard (telemetry + tunables)
        dashboard = FtcDashboard.getInstance();

        telemetry.addLine("Opening camera...");
        telemetry.update();

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                // Lower resolution & FPS for Control Hub stability
                webcam.startStreaming(160, 120, OpenCvCameraRotation.UPRIGHT);

                // If stable, uncomment next line to see live video stream:
                // dashboard.startCameraStream(webcam, 10);

                telemetry.addLine("Camera ready! Stream visible on Dashboard if enabled.");
                telemetry.update();
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Camera error", errorCode);
                telemetry.update();
            }
        });

        while (!isStarted() && !isStopRequested()) {
            telemetry.addLine("Open Dashboard: http://192.168.43.1:8080/dash");
            telemetry.addLine("Tune HSV to isolate your ball color");
            telemetry.update();
            sleep(50);
        }

        while (opModeIsActive()) {
            telemetry.addData("FPS", webcam.getFps());
            telemetry.addData("Detected X", detectedX);
            telemetry.addData("Detected Y", detectedY);
            telemetry.addData("Area", detectedArea);
            telemetry.update();
            sleep(100);
        }

        webcam.stopStreaming();
        webcam.closeCameraDevice();
    }

    // --- Color-based object detection pipeline ---
    public static class BallDetectPipeline extends OpenCvPipeline {
        private final Mat hsv = new Mat();
        private final Mat mask = new Mat();
        private final Mat hierarchy = new Mat();
        private final Mat output = new Mat();

        @Override
        public Mat processFrame(Mat input) {
            Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);

            // Use live Dashboard tunables
            Scalar lower = new Scalar(TestVision.hMin, TestVision.sMin, TestVision.vMin);
            Scalar upper = new Scalar(TestVision.hMax, TestVision.sMax, TestVision.vMax);

            Core.inRange(hsv, lower, upper, mask);

            // Find contours of filtered areas
            List<MatOfPoint> contours = new ArrayList<>();
            Imgproc.findContours(mask, contours, hierarchy,
                    Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

            // Copy the frame to draw on
            input.copyTo(output);

            double largestArea = 0;
            Rect largestRect = null;

            for (MatOfPoint contour : contours) {
                double area = Imgproc.contourArea(contour);
                if (area > largestArea) {
                    largestArea = area;
                    largestRect = Imgproc.boundingRect(contour);
                }
            }

            if (largestRect != null) {
                Imgproc.rectangle(output, largestRect.tl(), largestRect.br(), new Scalar(0, 255, 0), 2);
                Imgproc.putText(output,
                        String.format("Area: %.0f", largestArea),
                        largestRect.tl(),
                        Imgproc.FONT_HERSHEY_SIMPLEX, 0.5,
                        new Scalar(255, 255, 255), 1);

                // Save detection data for telemetry
                detectedX = largestRect.x + largestRect.width / 2.0;
                detectedY = largestRect.y + largestRect.height / 2.0;
                detectedArea = largestArea;
            } else {
                detectedX = -1;
                detectedY = -1;
                detectedArea = 0;
            }

            return output;
        }


        public void release() {
            hsv.release();
            mask.release();
            hierarchy.release();
            output.release();
        }
    }
}
