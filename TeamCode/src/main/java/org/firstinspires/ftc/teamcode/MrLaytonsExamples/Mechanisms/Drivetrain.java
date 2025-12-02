package org.firstinspires.ftc.teamcode.MrLaytonsExamples.Mechanisms;

import static android.os.SystemClock.sleep;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;



import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

public class Drivetrain {

    // Set up the motor variables used for the drivetrain. You need one for each motor used.
    // These should be private so they are only used by this class.
    private DcMotor leftMotor; // Use variable names that make sense to anyone who would read the code.
    private DcMotor rightMotor;

    private final ElapsedTime autoDriveTimer = new ElapsedTime();

    public void init(HardwareMap hwMap) { // Initialize our drivetrain motors.


        // This gets the information from the configuration from the file we made on the drivers station
        leftMotor = hwMap.get(DcMotor.class,"leftMotor"); // The name here - "exampleName" - MUST match what you called it in the config file on the drivers station.
        rightMotor = hwMap.get(DcMotor.class, "rightMotor");

        // Configure motors with any extra features, such as reversing direction to account for installation.
        rightMotor.setDirection(DcMotorSimple.Direction.REVERSE); // This reverses the standard direction of the motor when power is applied.
        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER); // This allows the motors to run using the built in encoders to track motor rotations.
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    // This is a drive control that uses the left thumb stick to control the robot, both forward/reverse and rotation.
    public void arcadeDrive ( double throttle, double rotation) {
        double leftPower = Range.clip(throttle + rotation,-1.0,1.0);
        double rightPower = Range.clip(throttle - rotation, -1.0,1.0);

        leftMotor.setPower(leftPower);
        rightMotor.setPower(rightPower);

        telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
        telemetry.update();
    }

    /**
     * For autonomous, the robot is using a timer and encoders on the drivetrain to move away from the target.
     * This method contains the math to be used with the inputted distance for the encoders, resets the elapsed timer, and
     * provides a check for it to run so long as the motors are busy and the timer has not run out.
     */
    public void autoDrive(double speed, int leftDistanceInch, int rightDistanceInch, int timeout_ms) {
        autoDriveTimer.reset();
        double WHEELS_INCHES_TO_TICKS = (28 * 5 * 3) / (3 * Math.PI);
        leftMotor.setTargetPosition((int) (leftMotor.getCurrentPosition() + leftDistanceInch * WHEELS_INCHES_TO_TICKS));
        rightMotor.setTargetPosition((int) (rightMotor.getCurrentPosition() + rightDistanceInch * WHEELS_INCHES_TO_TICKS));
        leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftMotor.setPower(Math.abs(speed));
        rightMotor.setPower(Math.abs(speed));
        while ((leftMotor.isBusy() || rightMotor.isBusy()) && autoDriveTimer.milliseconds() < timeout_ms) {
            sleep(50);
        }
        leftMotor.setPower(0);
        rightMotor.setPower(0);
        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

}
