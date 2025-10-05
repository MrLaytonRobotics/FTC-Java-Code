package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "EHS Testing", group = "Omni OpMode")
//    @Config // Enables configuration via FTC Dashboard
public class Testing extends LinearOpMode {

    private DcMotor backRightMotor = null;
    private DcMotor backLeftMotor = null;
    private DcMotor frontRightMotor = null;
    private DcMotor frontLeftMotor = null;
    //DcMotor fireWheel;


    public void runOpMode() {
        backRightMotor = hardwareMap.get(DcMotor.class, "back_right_drive");
        backLeftMotor = hardwareMap.get(DcMotor.class, "back_left_drive");
        frontRightMotor = hardwareMap.get(DcMotor.class, "front_right_drive");
        frontLeftMotor = hardwareMap.get(DcMotor.class, "front_left_drive");
        frontLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        backLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        frontRightMotor.setDirection(DcMotor.Direction.FORWARD);
        backRightMotor.setDirection(DcMotor.Direction.FORWARD);
        //fireWheel = hardwareMap.get(DcMotor.class, "fireWheel");
        double ly = -gamepad1.left_stick_y; // y stick is reversed
        double lx = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;
        waitForStart();

        while (opModeIsActive()) {
            frontLeftMotor.setPower(ly + lx + rx);
            backLeftMotor.setPower(ly - lx + rx);
            frontRightMotor.setPower(ly - lx - rx);
            backRightMotor.setPower(ly + lx - rx);
           // if (gamepad1.right_trigger > 0.1) {
                //fireWheel.setPower(1);
            //}
        }

    }
}
