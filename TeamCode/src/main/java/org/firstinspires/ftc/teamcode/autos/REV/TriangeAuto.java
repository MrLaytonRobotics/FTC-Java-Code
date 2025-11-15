package org.firstinspires.ftc.teamcode.autos.REV;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous
public class TriangeAuto extends LinearOpMode {
    private DcMotor leftDrive;
    private DcMotor rightDrive;

    @Override
    public void runOpMode() {
        leftDrive = hardwareMap.get(DcMotor.class, "leftDrive");
        rightDrive = hardwareMap.get(DcMotor.class, "rightDrive");

        rightDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        waitForStart();

        rightDrive.setPower(0.65);
        leftDrive.setPower(1);

        sleep(1000);
        leftDrive.setPower(0);
        rightDrive.setPower(0);
    }
}
