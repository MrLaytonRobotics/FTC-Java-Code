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
        boolean isOn;
        x = gamepad1.right_stick_x;
        y = -gamepad1.right_stick_y;
        isOn = gamepad1.a;


        //Values of gamepad
        telemetry.addData("X Value of the GamePad", x);
        telemetry.addData("Y Value of the GamePad", y);

        if(isOn) {
            //Drive Power
            telemetry.addData("LeftDrive Power", y - x);
            telemetry.addData("RightDrive Power", y + x);
            telemetry.addData("Is on", isOn);
            //Set the power
            leftDrive.setPower(y - x);
            rightDrive.setPower(y + x);
        }else{
            telemetry.addData("Is on", isOn);
            telemetry.addData("LeftDrive Power", 0);
            telemetry.addData("RightDrive Power", 0);
            //Set the power
            leftDrive.setPower(0);
            rightDrive.setPower(0);
        }

        telemetry.update();

    }
}
