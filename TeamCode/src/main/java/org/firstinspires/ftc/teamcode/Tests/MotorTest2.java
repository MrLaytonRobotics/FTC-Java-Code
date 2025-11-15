package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;



//Motor Test
@TeleOp
public class MotorTest2 extends LinearOpMode {
    private DcMotor rightDrive;
    private DcMotor leftDrive;

    @Override
    public void runOpMode() {
        rightDrive = hardwareMap.get(DcMotor.class, "rightDrive");
        leftDrive = hardwareMap.get(DcMotor.class, "leftDrive");

        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                rightDrive.setPower(1);
                leftDrive.setPower(-1);
            }
        }
    }
}