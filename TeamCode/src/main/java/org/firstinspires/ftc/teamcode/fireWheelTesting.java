package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "fireWheelTesting", group = "Test")
//    @Config // Enables configuration via FTC Dashboard
public class fireWheelTesting extends LinearOpMode {
    DcMotor fireWheel;


    public void runOpMode() {
        fireWheel = hardwareMap.get(DcMotor.class, "fireWheel");
        fireWheel.setDirection(DcMotor.Direction.FORWARD);
        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.a) {
                fireWheel.setPower(1.0);   // run motor full power
            } else {
                fireWheel.setPower(0.0);   // stop motor when not pressing A
            }
        }

    }
}
