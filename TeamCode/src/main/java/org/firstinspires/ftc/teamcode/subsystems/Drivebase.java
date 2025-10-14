package org.firstinspires.ftc.teamcode.subsystems;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Drivebase {
    private final DcMotorEx leftFrontDrive, leftBackDrive, rightFrontDrive, rightBackDrive;

    public Drivebase(HardwareMap hardwareMap) {
        leftFrontDrive = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftBackDrive = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightFrontDrive = hardwareMap.get(DcMotorEx.class, "rightFront");
        rightBackDrive = hardwareMap.get(DcMotorEx.class, "rightBack");

        leftFrontDrive.setDirection(DcMotorEx.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotorEx.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotorEx.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotorEx.Direction.FORWARD);

        leftFrontDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    }

    private final double speed = 1.00;
    private final double turn_speed = 1.00;
    private double max, axial, lateral, yaw;
    private double leftFrontPower, rightFrontPower, leftBackPower, rightBackPower;

    public void update(Gamepad gamepad1, Gamepad gamepad2) {
        axial = -gamepad1.left_stick_y + -(gamepad2.left_stick_y);  // Note: pushing stick forward gives negative value
        lateral = gamepad1.left_stick_x + (gamepad2.left_stick_x);
        yaw = (gamepad1.right_stick_x + (gamepad2.right_stick_x)) * turn_speed;

        // Combine the joystick requests for each axis-motion to determine each wheel's power.
        // Set up a variable for each drive wheel to save the power level for telemetry.
        leftFrontPower = axial + lateral + yaw;
        rightFrontPower = axial - lateral - yaw;
        leftBackPower = axial - lateral + yaw;
        rightBackPower = axial + lateral - yaw;

        // Normalize the values so no wheel power exceeds 100%
        // This ensures that the robot maintains the desired motion.
        max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        leftFrontPower *= speed;
        rightFrontPower *= speed;
        leftBackPower *= speed;
        rightBackPower *= speed;

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        // Send calculated power to wheels
        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);
    }
}