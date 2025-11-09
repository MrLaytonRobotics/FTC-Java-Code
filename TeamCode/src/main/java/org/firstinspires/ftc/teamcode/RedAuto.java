package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous
public class RedAuto extends LinearOpMode {
    private DcMotor leftDrive;
    private DcMotor rightDrive;

    @Override
    public void runOpMode() {
        leftDrive = hardwareMap.get(DcMotor.class, "leftDrive");
        rightDrive = hardwareMap.get(DcMotor.class, "rightDrive");

        long sleepingTime1 = 1000;
        long sleepingTime2 = 1000;

        rightDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        waitForStart();

        rightDrive.setPower(0.65);
        leftDrive.setPower(-1);
        sleep(sleepingTime1);
        telemetry.addData("Sleep ms 1", sleepingTime1);
        leftDrive.setPower(1);
        rightDrive.setPower(0.65);

        sleep(sleepingTime2);
        telemetry.addData("Sleep ms 2", sleepingTime2);

        leftDrive.setPower(0);
        rightDrive.setPower(0);


        telemetry.addLine("Auto Finished");
    }
}
