package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class GamePadTester extends LinearOpMode {
    @Override
    public void runOpMode() {
        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                telemetry.addData("The Left Stick X", gamepad1.left_stick_x);
                telemetry.addData("The Left Stick Y", gamepad1.left_stick_y);
                telemetry.update();
            }
        }
    }
}

