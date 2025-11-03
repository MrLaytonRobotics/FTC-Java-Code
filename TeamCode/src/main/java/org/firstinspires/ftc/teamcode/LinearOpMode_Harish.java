package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous
public class LinearOpMode_Harish extends com.qualcomm.robotcore.eventloop.opmode.LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        double fLeftPower, fRightPower, bLeftPower, bRightPower;
        DcMotor fLeftMotor = hardwareMap.get(DcMotor.class, "frontleft");
        DcMotor fRightMotor = hardwareMap.get(DcMotor.class, "frontright");
        DcMotor bLeftMotor = hardwareMap.get(DcMotor.class, "backleft");
        DcMotor bRightMotor = hardwareMap.get(DcMotor.class, "backright");

        // Positive power, move forward
        //fLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        fRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        bRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Coast to stop slowly, BRAKE will stop it immediately
        fLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        fRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        bLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        bRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        //RUN_USING_ENCODER will measure motor power using TICKS
        fLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //Wait for start/play button to be pressed on the driver hub
        waitForStart();

        double drive, turn, strafe;

        while(opModeIsActive()){
            fLeftMotor.setPower(1); // Full power with direction set initially

            drive = gamepad1.left_stick_y * -1;
            turn = gamepad1.right_stick_x;
            strafe = gamepad1.left_stick_x;

            fLeftPower = drive + turn;
            fRightPower = drive - turn;
            bLeftPower = drive + turn;
            bRightPower = drive - turn;


        }
    }
}
