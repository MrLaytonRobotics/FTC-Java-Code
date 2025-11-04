package org.firstinspires.ftc.teamcode.teleop;
//
//import com.arcrobotics.ftclib.controller.PIDController;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp
public class Teleop extends LinearOpMode {
//    private PIDController controller;
    double shooterX = 135;
    double shooterY = 135;
    public static double p = 0.025, i = 0, d = 0;
    public static double target = 0;
    public double hoodpos = 0;
    public static Follower follower;
    @Override
    public void runOpMode() throws InterruptedException {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(72, 72, 0));
        follower.startTeleopDrive();
        follower.update();
//        controller = new PIDController(p, i, d);
        // Declare our motors
        // Make sure your ID's match your configuration
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("lf");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("lb");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("rf");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("rb");
        DcMotorEx intake = hardwareMap.get(DcMotorEx.class, "intake");
        DcMotor turret = hardwareMap.dcMotor.get("turret");
        DcMotor shooterb = hardwareMap.dcMotor.get("sb");
        DcMotor shootert = hardwareMap.dcMotor.get("st");
//        CRServo transfer = hardwareMap.crservo.get("transfer");
        Servo hood = hardwareMap.servo.get("hood");
        Servo latch = hardwareMap.servo.get("latch");


        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        // See the note about this earlier on this page.
        frontLeftMotor.setDirection(DcMotorEx.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorEx.Direction.FORWARD);
        frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            intake.setPower(gamepad1.left_stick_x);
//            follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, true);
//            follower.update();
//
//            double robotX = follower.getPose().getX();
//            double robotY = follower.getPose().getY();
//            double robotHeading = follower.getPose().getHeading();
//            double distance = Math.sqrt(Math.pow(Math.abs(shooterX - robotX), 2) + Math.pow(Math.abs(shooterY - robotY), 2));
//            double theta = Math.atan(Math.abs(shooterY - robotY) / Math.abs(shooterX - robotX)); // radians
//            double turretAngle = theta - robotHeading;
//            double turretDegrees = Math.toDegrees(turretAngle);
//            boolean turretonoff = true;
//            turretDegrees = (turretDegrees + 180) % 360 / 360.0;
//            target = turretDegrees * 1635 + 25;
//            controller.setPID(p, i, d);
//            int pos = turret.getCurrentPosition();
//            double pid = controller.calculate(pos, target);
//            if (gamepad1.x) {
//                if (turretonoff = true) {
//                    turretonoff = false;
//                }
//                if (turretonoff = false) {
//                    turretonoff = true;
//                }
//            }
//            if (turretonoff = true) {
//                turret.setPower(-pid);
//            }
            if (gamepad1.a) {
                shootert.setPower(-1);
                shooterb.setPower(1);
            } else {
                shootert.setPower(0);
                shooterb.setPower(0);
            }
            hood.setPosition(hoodpos);
            if (gamepad1.y) {
                latch.setPosition(1);
            } else {
                latch.setPosition(0);
            }
//          flywheel.setVelocity(rpm * multiplier);

//            telemetry.addData("Turret angle: ", Math.toDegrees(turretAngle));
//            telemetry.addData("Distance: ", distance);
//            telemetry.addData("Distance: ", robotX);
//            telemetry.addData("Distance: ", robotY);
            telemetry.addData("right trigger", gamepad1.left_stick_y);
            telemetry.update();
        }
    }
}