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
        //This might be inverted when I last checked the code.
        x = gamepad1.right_stick_x;
        y = -gamepad1.left_stick_y;

        leftDrive.setPower(y - x);
        rightDrive.setPower(y + x);

    }
}
