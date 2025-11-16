package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx; // Import the DcMotorEx class
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
// Import for PIDF control

import java.util.List;

@TeleOp(name = "week2Limelightv3", group = "Competition")
//@Disabled
public class Week2Limelightv1 extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    // drive motors
    DcMotorEx frontLeft, frontRight, backLeft, backRight;
    //other motors
    DcMotorEx intake, shooter, transfer, turret;
    // limelight
    private Limelight3A limelight;

    // IMU
    private IMU imu;
    // Servos
    Servo hold;
    Servo arm;

    // --- State Variables for Toggles ---
    boolean shoot = false;
    boolean climbHold = false;
    boolean openHold = false;


    double distance = 0;

    private static double CAMERA_ANGLE_RADIANS_H_PLANE;

    private static final double CAMERA_HEIGHT_MM = 370.  ; // 380.0;
    // You must still define the target height in MM (e.g., center of a game-specific AprilTag)
    private static final double TARGET_HEIGHT_MM = 744; // 476.25; // 140.0; // Placeholder value, YOU MUST CHANGE THIS

    private static final int TARGET_TAG_ID = 20;

    boolean fieldBasedDriving = false;
    double forward, strafe, rotate;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initializing...");
        telemetry.update();

        // --- HARDWARE MAPPING ---
        frontLeft = hardwareMap.get(DcMotorEx.class, "FrontLeft");
        frontRight = hardwareMap.get(DcMotorEx.class, "FrontRight");
        backLeft = hardwareMap.get(DcMotorEx.class, "BackLeft");
        backRight = hardwareMap.get(DcMotorEx.class, "BackRight");

        intake = hardwareMap.get(DcMotorEx.class, "Intake");
        shooter = hardwareMap.get(DcMotorEx.class, "Shooter"); // Mapped as DcMotorEx to read encoder
        transfer = hardwareMap.get(DcMotorEx.class, "Transfer");
        turret = hardwareMap.get(DcMotorEx.class, "Turret");

        PIDFCoefficients shooterPIDF = new PIDFCoefficients(53, 0.02, 3, 11);
        shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, shooterPIDF);

        hold = hardwareMap.get(Servo.class, "hold");
        arm = hardwareMap.get(Servo.class, "kicker");
        telemetry.setMsTransmissionInterval(5);
        shooter.setPower(0);


        // --- MOTOR DIRECTION ---
        // drive motors
        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);
        intake.setDirection(DcMotor.Direction.REVERSE);
        shooter.setDirection(DcMotor.Direction.FORWARD);

        // motor behaviors

        // drive motors
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // other motrs
        transfer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //drive motor modes
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // other motor modes
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        transfer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //AprilTagWebcam aprilTagWebcam = new AprilTagWebcam();

        //a//prilTagWebcam.init(hardwareMap, telemetry);


        //create constants
        //max 1800
        double newTargetVelocity = 1600.0;


        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        telemetry.setMsTransmissionInterval(11);

        limelight.start();

        limelight.pipelineSwitch(0);

        CAMERA_ANGLE_RADIANS_H_PLANE = 0.1658;//Math.toRadians(25);//0.1658;

        // Now initialize the IMU with this mounting orientation
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot RevOrientation = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD);

        imu.initialize(new IMU.Parameters(RevOrientation));

        waitForStart();
        /*
        hold.setPosition(0.2);
        sleep(2000);
        hold.setPosition(0.45);
        sleep(2000);
*/
        runtime.reset();

        intake.setPower(1);

        while (opModeIsActive()) {

            if (fieldBasedDriving) {
                fieldBasedDrive();
            } else {
                macanumDrive();
            }
            //end drive code

            // aprilTagWebcam.update();
            //AprilTagDetection id20 = aprilTagWebcam.getTagBySpecificId(20);

            if (!openHold) {
                /*if (id20 != null) {
                    distance = id20.ftcPose.range;
                    telemetry.addData( "Distance ", distance);
                }*/

                LLResult result = limelight.getLatestResult();
                if (result.isValid()) {
                    List<LLResultTypes.FiducialResult>fiducialResults = result.getFiducialResults();
                    for (LLResultTypes.FiducialResult fr : fiducialResults) {
                        if(fr.getFiducialId() != TARGET_TAG_ID) continue;
                        distance = calculateDistance(Math.toRadians(result.getTy()), telemetry);
                        telemetry.addData(
                                "Fiducial", "ID: %d, Family: %s, X: %.2f, Y: %.2f, angle: %.2f",
                                fr.getFiducialId(), fr.getFamily(),
                                fr.getTargetXDegrees(), fr.getTargetYDegrees(),
                                result.getTy());
                    }
                }

                // if (distance > 55 && distance < 70) {
                //     newTargetVelocity = 1500 * (distance / 58.0);
                // }

                // else if (distance > 30 && distance < 45) {
                //     newTargetVelocity = 1300 * (distance / 38.0);
                // }
                // else if (distance > 40 && distance < 55) {
                //     newTargetVelocity = 1425 * (distance / 47.0);
                // }

                // else if (distance > 90 && distance < 110) {
                //     newTargetVelocity = 1900 * (distance / 105.0);
                // }
                //newTargetVelocity = 7.5 * distance + 1100;

                //newTargetVelocity = 0.4787 * distance + 707;// v1
                //if(distance<1200)
                  //  newTargetVelocity = 0.4787 * distance + 860;
                //else if(distance<2100)
                  //  newTargetVelocity = 0.4787 * distance + 660;
                //else
                  //  newTargetVelocity = 0.4787 * distance + 695;
                if (distance<2000)
                {
                    //newTargetVelocity=1021.43464*(Math.pow(1.000243766,distance));
                    newTargetVelocity = 0.3512* distance + 956.4;
                }
                else
                {
                     newTargetVelocity = 0.3512* distance + 1025;
                }
            }
            telemetry.addData( "Target velocity ", newTargetVelocity);
            telemetry.addData( "Distance ", distance);

            double prevX = 100;

            while(gamepad1.triangle && Math.abs(prevX) > 0.5) {
                LLResult result = limelight.getLatestResult();
                if (result.isValid()) {
                    List<LLResultTypes.FiducialResult>fiducialResults = result.getFiducialResults();
                    for (LLResultTypes.FiducialResult fr : fiducialResults) {
                        if(fr.getFiducialId() != TARGET_TAG_ID) continue;
                        prevX = fr.getTargetXDegrees();
                        telemetry.addData(
                                "Fiducial for direction", "ID: %d, X: %.2f, Y: %.2f, angle: %.2f",
                                fr.getFiducialId(), fr.getTargetXDegrees(),
                                fr.getTargetYDegrees(), result.getTy());
                        telemetry.update();
                        //double speed = prevX;
                        //while(Math.abs(speed) > 0.2) speed = speed / 10;
                        double speed;
                        if(Math.abs(prevX) > 15) speed = 0.3;
                        if(Math.abs(prevX) > 10) speed = 0.2;
                        else speed = 0.1;
                        if(prevX < 0) rotate(-1 * speed);
                        else rotate(speed);
                        break;
                    }
                }
                else break;
            }

            if ((gamepad2.a || gamepad1.a) )
            {

                //trying to fire
                if (shooter.getVelocity() >= newTargetVelocity)
                {
                    //good to shoot
                    shooter.setPower(0.75);// was 0.55
                    shoot = true;
                    //sleep(100);
                } else {
                    //need to spin up more
                    shoot = false;
                    shooter.setPower(1);
                }
            }
            else
            {
                // Stop the shooter when not spinning up
                shooter.setPower(0.4);
                shoot = false;
            }

            if(shooter.getVelocity() >= (newTargetVelocity - 100))
            {
                openHold = true;
            }
            else
            {
                openHold = false;
            }

            // shooting
            //|| (shoot && aligned && limelightid == targetId && gamepad2.b)
            if (openHold) {
                hold.setPosition(0.2);
                sleep(100);
                transfer.setPower(0.8);
                intake.setPower(-1);
                //sleep(50);
                //arm.setPosition(1);
            }
            else {
                hold.setPosition(0.45);// it was 0.82
                //arm.setPosition(0);
            }

            if(gamepad1.dpad_down) newTargetVelocity++;
            if(gamepad1.dpad_up) newTargetVelocity--;

            if ((gamepad2.x || gamepad1.x || (shoot && (gamepad2.b || gamepad1.b))) ) {
                transfer.setPower(1);
                intake.setPower(-1);

            } else {


                if (gamepad1.right_trigger > 0.1) {
                    intake.setPower(-1);
                    transfer.setPower(1);
                } else if (gamepad1.left_trigger > 0.1) {
                    intake.setPower(1);
                    transfer.setPower(-1);

                } else {
                    if (!openHold) {
                        transfer.setPower(0);
                        intake.setPower(0);
                    }
                }
            }

            if (gamepad1.left_bumper) {
                fieldBasedDriving = true;
            } else if (gamepad1.right_bumper) {
                fieldBasedDriving = false;
            }
            //climber
            if(gamepad1.dpad_up || gamepad2.dpad_up)
            {
                turret.setPower(1);
            }
            else if (gamepad1.dpad_down || gamepad2.dpad_down)
            {
                climbHold = true;
            }else if (gamepad1.dpad_left) {
                turret.setPower(-0.1);
            }    else
            {
                if (climbHold)
                {
                    turret.setPower(0.5);
                } else
                {
                    turret.setPower(0.0);
                }


            }

            // telemetry
            telemetry.addData("Status", "Initialized");
            telemetry.addData("--- Shooter ---", "");
            telemetry.addData("Actual Velocity", "%.2f", shooter.getVelocity());
            telemetry.addData("Shooter Power", "%.2f", shooter.getPower());
            telemetry.addData("Open Hold", openHold);
            telemetry.addData("FieldBasedDriving", fieldBasedDriving);
            telemetry.update();
        }
    }

    private void fieldBasedDrive () {
        forward = gamepad1.left_stick_y;
        strafe = gamepad1.left_stick_x;
        rotate = gamepad1.right_stick_x;

        double theta = Math.atan2(forward, strafe);
        double r = Math.hypot(strafe, forward);

        theta = AngleUnit.normalizeRadians(theta - imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));
        double newForward = r * Math.sin(theta);
        double newStrafe = r * Math.cos(theta);

        this.drive(newForward, newStrafe, rotate, /*maxPower*/ 1.0, /*maxSpeed*/ 1.0);
    }

    private void drive (double forward, double strafe, double rotate, double maxPower, double maxSpeed) {
        double frontLeftPower = forward + strafe + rotate;
        double frontRightPower = forward - strafe - rotate;
        double backLeftPower = forward - strafe + rotate;
        double backRightPower = forward + strafe - rotate;

        maxPower = Math.max(maxPower, Math.abs(frontLeftPower));
        maxPower = Math.max(maxPower, Math.abs(frontRightPower));
        maxPower = Math.max(maxPower, Math.abs(backLeftPower));
        maxPower = Math.max(maxPower, Math.abs(backRightPower));

        frontLeft.setPower(maxSpeed * (frontLeftPower / maxPower));
        frontRight.setPower(maxSpeed * (frontRightPower / maxPower));
        backLeft.setPower(maxSpeed * (backLeftPower / maxPower));
        backRight.setPower(maxSpeed * (backRightPower / maxPower));

    }

    private void macanumDrive() {
        ///drive code
        double y = gamepad1.left_stick_y;  // Forward
        double x = -gamepad1.left_stick_x;  // Strafe
        double rx = gamepad1.right_stick_x;  // Rotate

        double frontLeftPower = y + x + rx;
        double backLeftPower = y - x + rx;
        double frontRightPower = y - x - rx;
        double backRightPower = y + x - rx;

        // Optional: Normalize powers if exceeding 1.0
        double max = Math.max(Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower)),
                Math.max(Math.abs(backLeftPower), Math.abs(backRightPower)));
        if (max > 1.0) {
            frontLeftPower /= max;
            backLeftPower /= max;
            frontRightPower /= max;
            backRightPower /= max;
        }

        frontLeft.setPower(frontLeftPower);
        backLeft.setPower(backLeftPower);
        frontRight.setPower(frontRightPower);
        backRight.setPower(backRightPower);
    }

    private double calculateDistance(double targetYAngleRadians, Telemetry telemetry) {
        // Total angle from the horizontal plane to the top of the target
        // Note: The Limelight SDK's getTy() value is relative to the camera's crosshair.
        // We add our physical camera angle to it.

        //Limelight distance eq: d = (h2-h1)/sin(a1 + a2) ?
        double totalAngleRadians = CAMERA_ANGLE_RADIANS_H_PLANE + targetYAngleRadians;

        // Use the formula: distance = (h2 - h1) / tan(angle)
        double currentDistance = (TARGET_HEIGHT_MM - CAMERA_HEIGHT_MM) / Math.sin(totalAngleRadians);

        telemetry.addData("BBB1", totalAngleRadians);
        telemetry.addData("BBB2", Math.sin(totalAngleRadians));
        telemetry.addData("BBB3", (TARGET_HEIGHT_MM - CAMERA_HEIGHT_MM));

        return currentDistance;
    }

    private void rotate(double rx) {
        double frontLeftPower = rx;
        double backLeftPower = rx;
        double frontRightPower = 0 - rx;
        double backRightPower = 0 - rx;

        double max = Math.max(Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower)),
                Math.max(Math.abs(backLeftPower), Math.abs(backRightPower)));

        if (max > 1.0) {
            frontLeftPower /= max;
            backLeftPower /= max;
            frontRightPower /= max;
            backRightPower /= max;
        }

        frontLeft.setPower(frontLeftPower);
        backLeft.setPower(backLeftPower);
        frontRight.setPower(frontRightPower);
        backRight.setPower(backRightPower);
    }
}

