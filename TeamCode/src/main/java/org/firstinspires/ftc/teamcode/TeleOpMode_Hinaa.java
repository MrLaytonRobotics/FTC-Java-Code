package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;



/* Copyright (c) 2021 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */


/*
 * 1) Axial:    Driving forward and backward               Left-joystick Forward/Backward
 * 2) Lateral:  Strafing right and left                     Left-joystick Right and Left
 * 3) Yaw:      Rotating Clockwise and counter clockwise    Right-joystick Right and Left
 *
 * This code is written assuming that the right-side motors need to be reversed for the robot to drive forward.
 * When you first test your robot, if it moves backward when you push the left stick forward, then you must flip
 * the direction of all 4 motors (see code below).
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@TeleOp(name="TeleOpMode_Hinaa", group="OpMode")
public class TeleOpMode_Hinaa extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor frontleft = null;
    private DcMotor backleft = null;
    private DcMotor frontright = null;
    private DcMotor backright = null;
    private DcMotor intake;
    private DcMotor outtakeleft;
    private DcMotor outtakeright;


    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        frontleft = hardwareMap.get(DcMotor.class, "frontleft");
        backleft = hardwareMap.get(DcMotor.class, "backleft");
        frontright = hardwareMap.get(DcMotor.class, "frontright");
        backright = hardwareMap.get(DcMotor.class, "backright");
        // Map motors to config names in the RC configuration
        intake = hardwareMap.get(DcMotor.class, "intake");
        outtakeleft = hardwareMap.get(DcMotor.class, "outtakeleft");
        outtakeright = hardwareMap.get(DcMotor.class, "outtakeright");

        // Set directions (one of the outtakes should spin opposite)
        outtakeleft.setDirection(DcMotor.Direction.FORWARD);
        outtakeright.setDirection(DcMotor.Direction.REVERSE);





        // ########################################################################################
        // !!!            IMPORTANT Drive Information. Test your motor directions.            !!!!!
        // ########################################################################################
        // Most robots need the motors on one side to be reversed to drive forward.
        // The motor reversals shown here are for a "direct drive" robot (the wheels turn the same direction as the motor shaft)
        // If your robot has additional gear reductions or uses a right-angled drive, it's important to ensure
        // that your motors are turning in the correct direction.  So, start out with the reversals here, BUT
        // when you first test your robot, push the left joystick forward and observe the direction the wheels turn.
        // Reverse the direction (flip FORWARD <-> REVERSE ) of any wheel that runs backward
        // Keep testing until ALL the wheels move the robot forward when you push the left joystick forward.
        frontleft.setDirection(DcMotor.Direction.REVERSE);
        backleft.setDirection(DcMotor.Direction.REVERSE);
        frontright.setDirection(DcMotor.Direction.FORWARD);
        backright.setDirection(DcMotor.Direction.FORWARD);

        // Stop motors initially
        intake.setPower(0);
        outtakeleft.setPower(0);
        outtakeright.setPower(0);

        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Initialized");
        telemetry.update();


        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double max;

            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial   = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double lateral =  gamepad1.left_stick_x;
            double yaw     =  gamepad1.right_stick_x;

            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            double frontleftPower  = axial + lateral + yaw;
            double frontrightPower = axial - lateral - yaw;
            double backleftPower   = axial - lateral + yaw;
            double backrightPower  = axial + lateral - yaw;

            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = Math.max(Math.abs(frontleftPower), Math.abs(frontrightPower));
            max = Math.max(max, Math.abs(backleftPower));
            max = Math.max(max, Math.abs(backrightPower));

            if (max > 1.0) {
                frontleftPower  /= max;
                frontrightPower /= max;
                backleftPower   /= max;
                backrightPower  /= max;
            }

            // This is test code:
            //
            // Uncomment the following code to test your motor directions.
            // Each button should make the corresponding motor run FORWARD.
            //   1) First get all the motors to take to correct positions on the robot
            //      by adjusting your Robot Configuration if necessary.
            //   2) Then make sure they run in the correct direction by modifying the
            //      the setDirection() calls above.
            // Once the correct motors move in the correct direction re-comment this code.

            /*
            frontLeftPower  = gamepad1.x ? 1.0 : 0.0;  // X gamepad
            backLeftPower   = gamepad1.a ? 1.0 : 0.0;  // A gamepad
            frontRightPower = gamepad1.y ? 1.0 : 0.0;  // Y gamepad
            backRightPower  = gamepad1.b ? 1.0 : 0.0;  // B gamepad
            */

            // Intake control — use right trigger for intake in, left trigger for reverse
            double intakePower = gamepad2.right_trigger - gamepad2.left_trigger;
            intake.setPower(intakePower);

            // Outtake control — press 'A' to run both outtakes, 'B' to reverse
            double outtakePower = 0;
            if (gamepad2.a) {
                outtakePower = .75; // full forward
            } else if (gamepad2.b) {
                outtakePower = -.75; // reverse
            }

            outtakeleft.setPower(outtakePower);
            outtakeright.setPower(outtakePower);

            // Send calculated power to wheels
            frontleft.setPower(frontleftPower);
            frontright.setPower(frontrightPower);
            backleft.setPower(backleftPower);
            backright.setPower(backrightPower);

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", frontleftPower, frontrightPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", backleftPower, backrightPower);
            telemetry.addData("Intake Power", intakePower);
            telemetry.addData("Outtake Power", outtakePower);
            telemetry.update();
        }
    }}
