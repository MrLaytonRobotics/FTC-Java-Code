package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.teamcode.directives.DefaultDrivebase;

import org.firstinspires.ftc.teamcode.stellarstructure.Subsystem;

public final class Drivebase extends Subsystem {
    private static final Drivebase drivebase = new Drivebase();

    public static Drivebase getInstance() {
        return drivebase;
    }

    private Drivebase() {}

    private final static double CARDINAL_SPEED = 0.70;
    private final static double TURN_SPEED = 0.70;

    private DcMotorEx leftFrontDrive, leftBackDrive, rightFrontDrive, rightBackDrive;

    @Override
    public void init(HardwareMap hardwareMap) {
        //todo: make omni wheel directive
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

    @Override
    public void setGamepads(Gamepad gamepad1, Gamepad gamepad2) {
        setDefaultDirective(new DefaultDrivebase(this, gamepad1, CARDINAL_SPEED, TURN_SPEED));
    }

    @Override
    public void update() {}

    public void setDrivePower(double leftFrontPower, double rightFrontPower, double leftBackPower, double rightBackPower) {
        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);
    }

	@Override
    public String getTelemetryData() {
        return String.format("Left Front: %.2f\nRight Front: %.2f\nLeft Back: %.2f\nRight Back: %.2f",
                leftFrontDrive.getPower(),
                rightFrontDrive.getPower(),
                leftBackDrive.getPower(),
                rightBackDrive.getPower());
    }
}