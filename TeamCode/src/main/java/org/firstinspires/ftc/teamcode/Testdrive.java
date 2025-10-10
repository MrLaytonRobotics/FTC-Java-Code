package org.firstinspires.ftc.teamcode;

import android.os.Debug;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "TestdriveOpMode Class", group = "rev0/tests")
//@Disabled
public class Testdrive extends OpMode {
    // THIS IS where we put our public variables that are stored in the oppmodes class

    public double LauncherMaxsSpeed = 1128; // The speed that the launcher moater wll spinn at when ready, and the speed that we want when it puts the ball into the chanmber
    public double LauncherSpeedLaunch = 1100; // the speed at witch the launcher will deplay the ball

    final double STOP_SPEED = 0.0; //We send this power to the servos when we want them to stop.
    final double FULL_SPEED = 1.0;
    //THIS IS where we declare our OpMode Mappnigs
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotorEx launcher = null;
    private CRServo leftFeeder = null;
    private CRServo rightFeeder = null;



    @Override
    public void init() {
        // this code runs when the user preses start !ONCE!

        // Initalize Hardware Mappings
        leftDrive = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        leftFeeder = hardwareMap.get(CRServo.class, "left_feeder");
        rightFeeder = hardwareMap.get(CRServo.class, "right_feeder");
        // then set deractions and stuff
        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);
        launcher.setDirection(DcMotor.Direction.FORWARD);
        leftFeeder.setDirection(CRServo.Direction.FORWARD);
        rightFeeder.setDirection(CRServo.Direction.REVERSE);
        launcher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void loop() {
        // this is where you putt all the code for the robots op mode when it is initalized

        if (gamepad1.a)
        {
           launcher.setVelocity(LauncherMaxsSpeed);
            if (Math.abs(launcher.getVelocity()) >= LauncherSpeedLaunch)
            {

                leftFeeder.setPower(FULL_SPEED);
                rightFeeder.setPower(FULL_SPEED);
            }
        } else {
            launcher.setVelocity(0);
            leftFeeder.setPower(0);
            rightFeeder.setPower(0);
        }


        telemetry.addData("motorSpeed", Math.abs(launcher.getVelocity()));

    }

    //CUSTOM METHODS

}
