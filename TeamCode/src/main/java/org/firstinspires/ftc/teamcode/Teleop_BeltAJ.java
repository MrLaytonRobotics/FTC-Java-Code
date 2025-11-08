package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="TeleOpMode_withBelt", group="OpMode")
public class Teleop_BeltAJ extends LinearOpMode {

    // Declare motors
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor frontleft = null;
    private DcMotor backleft = null;
    private DcMotor frontright = null;
    private DcMotor backright = null;
    private DcMotor intake;
    private DcMotor outtakeleft;
    private DcMotor outtakeright;

    // --- Added servo conveyor ---
    private Servo belt;
    private boolean continuousMode = true;
    private boolean servoRunning = false;
    private long servoStartTime;

    @Override
    public void runOpMode() {

        // Initialize motors
        frontleft = hardwareMap.get(DcMotor.class, "frontleft");
        backleft = hardwareMap.get(DcMotor.class, "backleft");
        frontright = hardwareMap.get(DcMotor.class, "frontright");
        backright = hardwareMap.get(DcMotor.class, "backright");
        intake = hardwareMap.get(DcMotor.class, "intake");
        outtakeleft = hardwareMap.get(DcMotor.class, "outtakeleft");
        outtakeright = hardwareMap.get(DcMotor.class, "outtakeright");

        // --- Map servo in configuration ---
        belt = hardwareMap.get(Servo.class, "conveyorServo");

        // Servo neutral (stop for continuous rotation servo)
        belt.setPosition(0.5);

        // Directions
        outtakeleft.setDirection(DcMotor.Direction.FORWARD);
        outtakeright.setDirection(DcMotor.Direction.REVERSE);

        frontleft.setDirection(DcMotor.Direction.REVERSE);
        backleft.setDirection(DcMotor.Direction.REVERSE);
        frontright.setDirection(DcMotor.Direction.FORWARD);
        backright.setDirection(DcMotor.Direction.FORWARD);

        intake.setPower(0);
        outtakeleft.setPower(0);
        outtakeright.setPower(0);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {

            double axial   = -gamepad1.left_stick_y;
            double lateral =  gamepad1.left_stick_x;
            double yaw     =  gamepad1.right_stick_x;

            double frontleftPower  = axial + lateral + yaw;
            double frontrightPower = axial - lateral - yaw;
            double backleftPower   = axial - lateral + yaw;
            double backrightPower  = axial + lateral - yaw;

            double max = Math.max(Math.max(Math.abs(frontleftPower), Math.abs(frontrightPower)),
                    Math.max(Math.abs(backleftPower), Math.abs(backrightPower)));

            if (max > 1.0) {
                frontleftPower  /= max;
                frontrightPower /= max;
                backleftPower   /= max;
                backrightPower  /= max;
            }

            // Drive wheels
            frontleft.setPower(frontleftPower);
            frontright.setPower(frontrightPower);
            backleft.setPower(backleftPower);
            backright.setPower(backrightPower);

            // Intake
            double intakePower = gamepad2.right_trigger - gamepad2.left_trigger;
            intake.setPower(intakePower);

            // Outtake
            double outtakePower = 0;
            if (gamepad2.a) outtakePower = 1.0;
            else if (gamepad2.b) outtakePower = -1.0;

            outtakeleft.setPower(outtakePower);
            outtakeright.setPower(outtakePower);

            // --- Conveyor Servo Control ---
            // X = toggle mode (continuous <-> timed)
            if (gamepad2.x && !servoRunning) {
                continuousMode = !continuousMode;
                telemetry.addData("Conveyor Mode", continuousMode ? "Continuous" : "Timed");
            }

            // Y = activate conveyor
            if (gamepad2.y && !servoRunning) {
                servoRunning = true;
                servoStartTime = System.currentTimeMillis();

                if (continuousMode) {
                    belt.setPosition(1.0); // full forward
                } else {
                    belt.setPosition(1.0); // start timed run
                }
            }

            // Stop servo when button released (continuous mode)
            if (!gamepad2.y && continuousMode && servoRunning) {
                belt.setPosition(0.5); // neutral stop
                servoRunning = false;
            }

            // Auto-stop after duration (timed mode)
            if (!continuousMode && servoRunning &&
                    (System.currentTimeMillis() - servoStartTime) > 2000) { // 2 seconds
                belt.setPosition(0.5);
                servoRunning = false;
            }

            telemetry.addData("Status", "Run Time: " + runtime);
            telemetry.addData("Conveyor Mode", continuousMode ? "Continuous" : "Timed");
            telemetry.addData("Conveyor Running", servoRunning);
            telemetry.update();
        }
    }
}
