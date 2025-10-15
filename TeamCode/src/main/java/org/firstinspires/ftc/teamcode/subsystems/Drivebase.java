package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;

public final class Drivebase {
    private final double cardinalSpeed;
    private final double turnSpeed;
    private final DcMotorEx leftFrontDrive, leftBackDrive, rightFrontDrive, rightBackDrive;

    public Drivebase(double cardinalSpeed, double turnSpeed, HardwareMap hardwareMap) {
        this.cardinalSpeed = cardinalSpeed;
        this.turnSpeed = turnSpeed;

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

    public void update(Gamepad gamepad) {
        double max, axial, lateral, yaw;
        double leftFrontPower, rightFrontPower, leftBackPower, rightBackPower;
        axial = -gamepad.left_stick_y * cardinalSpeed;  // Note: pushing stick forward gives negative value
        lateral = gamepad.left_stick_x * cardinalSpeed;
        yaw = gamepad.right_stick_x * turnSpeed;

        // combine the joystick requests for each axis-motion to determine each wheel's power
        leftFrontPower = axial + lateral + yaw;
        rightFrontPower = axial - lateral - yaw;
        leftBackPower = axial - lateral + yaw;
        rightBackPower = axial + lateral - yaw;

        // normalize the values so no wheel power exceeds 100%
        // this ensures that the robot maintains the desired motion
        max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        // maintain desired motion
        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        // send calculated power to wheels
        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);
    }

    public String getTelemetryData() {
        return String.format("Left Front: %f\nRight Front: %f\nLeft Back: %f\nRight Back: %f", leftFrontDrive.getPower(), rightFrontDrive.getPower(), leftBackDrive.getPower(), rightBackDrive.getPower());
    }
}