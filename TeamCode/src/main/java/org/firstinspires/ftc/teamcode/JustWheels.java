package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class JustWheels extends OpMode {
    private DcMotor leftDrive;
    private DcMotor rightDrive;
    public void init() {
        leftDrive = hardwareMap.get(DcMotor.class, "leftDrive");
        rightDrive = hardwareMap.get(DcMotor.class, "rightDrive");
    }
    public void loop() {
        float x;
        float y;
        x = gamepad1.right_stick_x;
        y = -gamepad1.right_stick_y;

        //Values of gamepad
        telemetry.addData("X Value of the GamePad", x);
        telemetry.addData("Y Value of the GamePad", y);
        //Drive Power
        telemetry.addData("LeftDrive Power", y-x);
        telemetry.addData("RightDrive Power", y+x);
        //Set the power
        leftDrive.setPower(y - x);
        rightDrive.setPower(y + x);

    }
}
