package org.firstinspires.ftc.teamcode.MrLaytonsExamples.OpModes.TeleOp;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.MrLaytonsExamples.Mechanisms.Drivetrain;


// You must have this "annotation" below for your program to show up on the drivers station.
@TeleOp // Options are "@TeleOp" or "@Autonomous" this wil change where the program is called from on the driver station.

public class MyTeleOp extends OpMode { // You can not just call it TeleOp, that is already used by the system and you will override the original class and cause issues.
    Drivetrain drivetrain = new Drivetrain();
    double throttle, rotation;

    @Override
    public void init() {

        // This is where we create a new "Instance of the hardware we want use in our program
        drivetrain.init(hardwareMap);
    }

    @Override
    public void loop() {

        // This is where the code goes that we want to run while the play button has been pressed.

        // This uses the left thumbstick only to control the robot
        throttle = -gamepad1.left_stick_y;
        rotation = gamepad1.left_stick_x;

        // Calls our drive method we want to use.
        drivetrain.arcadeDrive(throttle,rotation);

    }
}
