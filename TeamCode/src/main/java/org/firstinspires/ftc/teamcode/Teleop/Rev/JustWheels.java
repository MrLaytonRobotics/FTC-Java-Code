package org.firstinspires.ftc.teamcode.Teleop.Rev;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;





@TeleOp
public class JustWheels extends OpMode {
    private DcMotor leftDrive;
    double maxSpeed = 0.65;
    private DcMotor rightDrive;

    public void init() {
        leftDrive = hardwareMap.get(DcMotor.class, "leftDrive");
        rightDrive = hardwareMap.get(DcMotor.class, "rightDrive");
    }
    public void loop() {
        double x;
        double y;

        /*if(gamepad1.a) {
            maxSpeed += 0.0001f;
        }
        if(gamepad1.b) {
            maxSpeed -= 0.0001f;
        }*/
        telemetry.addLine(String.valueOf(gamepad1.right_stick_x));
        telemetry.addLine(String.valueOf(gamepad1.left_stick_y));
        telemetry.update();


        y = gamepad1.right_stick_x;
        x = gamepad1.left_stick_y;
        leftDrive.setPower(y - x);
        rightDrive.setPower(Range.clip(y + x, -maxSpeed, maxSpeed));
    }
    float sign(float value) {
        if (value > 0) {
            return 1;
        } else if (value == 0) {
            return 0;
        } else {
            return -1;
        }
    }
}
