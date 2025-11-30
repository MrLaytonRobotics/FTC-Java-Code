package org.firstinspires.ftc.teamcode.MrLaytonsExamples.Hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Drivetrain {

    // Set up the motor variables used for the drivetrain. You need one for each motor used.
    // These should be private so they are only used by this class.
    private DcMotor leftMotor; // Use variable names that make sense to anyone who would read the code.
    private DcMotor rightMotor;

    public void init(HardwareMap hwMap) { // Initialize our drivetrain motors.


        // This gets the information from the configuration from the file we made on the drivers station
        leftMotor = hwMap.get(DcMotor.class,"leftMotor"); // The name here - "exampleName" - MUST match what you called it in the config file on the drivers station.
        rightMotor = hwMap.get(DcMotor.class, "rightMotor");

        // Configure motors with any extra features, such as reversing direction to account for installation.
        rightMotor.setDirection(DcMotorSimple.Direction.REVERSE); // This reverses the standard direction of the motor when power is applied.
        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER); // This allows the motors to run using the built in encoders to track motor rotations.
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }



}
