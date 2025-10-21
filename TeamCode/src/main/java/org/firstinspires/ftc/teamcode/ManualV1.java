package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import java.util.Arrays;

/**
 * TBD
 */
@TeleOp(name="Manual V1 (Swyft)", group="Iterative Opmode")
public class ManualV1 extends OpMode {

    // Program constants
    private static final double INCREMENT = 0.001;

    private DcMotor leftFront  = null;
    private DcMotor leftBack  = null;
    private DcMotor rightFront = null;
    private DcMotor rightBack = null;

    private DcMotor leftShooter = null;

    private DcMotor rightShooter = null;

    private DcMotor intake = null;
    // If true, only prints telemetry, nothing moves.
    private boolean dryRun = true;
    // Current speed
    double[] speeds = new double[4]; // LF, RF, LB, RB
    double leftShooterPower, rightShooterPower, intakePower;

    double maxMoveSpeed=0.3, maxShootPower =1.0, maxIntakePower =1.0;

    @Override
    public void init() {
        // Strings must match up with the config on the Robot Controller.
        leftFront   = hardwareMap.get(DcMotor.class, "left_front");
        rightFront   = hardwareMap.get(DcMotor.class, "right_front");
        leftBack  = hardwareMap.get(DcMotor.class, "left_back");
        rightBack  = hardwareMap.get(DcMotor.class, "right_back");

        leftShooter  = hardwareMap.get(DcMotor.class, "left_shooter");
        rightShooter  = hardwareMap.get(DcMotor.class, "right_shooter");

        rightShooter.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);
        intake  = hardwareMap.get(DcMotor.class, "intake");
    }

    void doTelemetry(double[] speeds) {
        telemetry.addData("DryRun", dryRun);
        telemetry.addData("How to start",   "L1 = TestOnly, L2 = Run");
        telemetry.addData(
                "Max Move Speed",   "Move %.2f (DPAD_UP ↑, DPAD_DOWN ↓)", maxMoveSpeed);
        //:: Shoot %.2f :: Intake %.2f", , maxShootPower, maxIntakePower

        telemetry.addData("Front", "LF ❂=%.2f ::: RF ❂=%.2f", speeds[0], speeds[1]);
        telemetry.addData("Back", "LB ❂=%.2f ::: RB ❂=%.2f ", speeds[2], speeds[3]);

        telemetry.addLine();
        telemetry.addData("Shooter", "Left=%.2f :::: Right=%.2f (X↑, Y↓)", leftShooterPower, rightShooterPower);
        telemetry.addData("Intake", "%.2f\t(A↑, B↓)",  intakePower);

        telemetry.addLine();
        telemetry.addData("LeftStick", "X=%.2f, Y=%.2f", gamepad1.left_stick_x, gamepad1.left_stick_y);
        telemetry.addData("RightStick", "X=%.2f, Y=%.2f", gamepad1.right_stick_x, gamepad1.right_stick_y);

        telemetry.update();
    }

    @Override
    public void loop() {
        if (!dryRun) {
            speeds = drive();
            leftShooter.setPower(leftShooterPower);
            rightShooter.setPower(rightShooterPower);
            intake.setPower(intakePower);
        }

        if (gamepad1.right_bumper){
            dryRun = false;
            Arrays.fill(speeds, 0); // Avoid surprises
            intakePower = leftShooterPower = rightShooterPower = 0;
        } else if (gamepad1.left_bumper) {
            dryRun = true;
            Arrays.fill(speeds, 0); // Avoid surprises
            intakePower = leftShooterPower = rightShooterPower = 0;
        } else if (gamepad1.x) {
            leftShooterPower += INCREMENT;
            leftShooterPower = Math.min(leftShooterPower, maxShootPower);

            rightShooterPower += INCREMENT;
            rightShooterPower = Math.min(rightShooterPower, maxShootPower);
        } else if (gamepad1.y) {
            leftShooterPower -= INCREMENT;
            leftShooterPower = Math.max(leftShooterPower, 0);

            rightShooterPower -= INCREMENT;
            rightShooterPower = Math.max(rightShooterPower, 0);
        } else if (gamepad1.a) {
            intakePower += INCREMENT;
            intakePower = Math.min(intakePower, maxIntakePower);
        } else if (gamepad1.b) {
            intakePower -= INCREMENT;
            intakePower = Math.max(intakePower, -maxIntakePower);
        } else if (gamepad1.dpad_right) {
            maxMoveSpeed += INCREMENT;
            maxMoveSpeed = Math.min(maxMoveSpeed, 1);
        } else if (gamepad1.dpad_down) {
            maxMoveSpeed -= INCREMENT;
            maxMoveSpeed = Math.max(maxMoveSpeed, 0);
        }

        doTelemetry(speeds);
    }

    double[] drive() {
        double drive  = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double twist  = gamepad1.right_stick_x;

        double[] speeds = {
                (drive + strafe + twist),
                (drive - strafe - twist),
                (drive - strafe + twist),
                (drive + strafe - twist)
        };

        double max = Math.abs(maxMoveSpeed);
        for (double v : speeds) {
            if (max < Math.abs(v)) max = Math.abs(v);
        }

        if (max > 1) {
            for (int i = 0; i < speeds.length; i++) speeds[i] /= max;
        }

        // apply the calculated values to the motors.
        leftFront.setPower(speeds[0]);
        rightFront.setPower(speeds[1]);
        leftBack.setPower(speeds[2]);
        rightBack.setPower(speeds[3]);
        return speeds;
    }
}

