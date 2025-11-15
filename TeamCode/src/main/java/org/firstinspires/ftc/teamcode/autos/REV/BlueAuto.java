package org.firstinspires.ftc.teamcode.autos.REV;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous
public class BlueAuto extends LinearOpMode {
    private DcMotor leftDrive;
    private DcMotor rightDrive;

    @Override
    public void runOpMode() {
        leftDrive = hardwareMap.get(DcMotor.class, "leftDrive");
        rightDrive = hardwareMap.get(DcMotor.class, "rightDrive");

        rightDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        waitForStart();

        rightDrive.setPower(-0.65);
        leftDrive.setPower(1);
        sleep(400);
        leftDrive.setPower(1);
        rightDrive.setPower(0.65);
        sleep(900);
        leftDrive.setPower(0);
        rightDrive.setPower(0);
    }
}
