package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous
        (name = "Auto_Amulya", group = "Autonomous")
public class Auto_Amulya extends LinearOpMode {

    // Drive motors
    private DcMotor frontleft, frontright, backleft, backright;
    // Intake/outtake motors
    private DcMotor intake, outtakeleft, outtakeright;
    private ElapsedTime runtime = new ElapsedTime();

    // Constants
    static final double COUNTS_PER_MOTOR_REV = 383.6; // goBILDA 20:1
    static final double DRIVE_GEAR_REDUCTION = 1.0;
    static final double WHEEL_DIAMETER_INCHES = 4.0;
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * Math.PI);
    static final double DRIVE_SPEED = 0.6;

    @Override
    public void runOpMode() {

        // ---- Initialize Drive Motors ----
        frontleft = hardwareMap.get(DcMotor.class, "frontleft");
        frontright = hardwareMap.get(DcMotor.class, "frontright");
        backleft = hardwareMap.get(DcMotor.class, "backleft");
        backright = hardwareMap.get(DcMotor.class, "backright");

        frontleft.setDirection(DcMotor.Direction.REVERSE);
        backleft.setDirection(DcMotor.Direction.REVERSE);
        frontright.setDirection(DcMotor.Direction.FORWARD);
        backright.setDirection(DcMotor.Direction.FORWARD);

        resetAndRunEncoders(frontleft, frontright, backleft, backright);

        // ---- Initialize Intake / Outtake Motors ----
        intake = hardwareMap.get(DcMotor.class, "intake");
        outtakeleft = hardwareMap.get(DcMotor.class, "outtakeleft");
        outtakeright = hardwareMap.get(DcMotor.class, "outtakeright");

        intake.setDirection(DcMotor.Direction.FORWARD); // change if motor spins wrong way
        outtakeleft.setDirection(DcMotor.Direction.FORWARD);
        outtakeright.setDirection(DcMotor.Direction.REVERSE);

        intake.setPower(0);
        outtakeleft.setPower(0);
        outtakeright.setPower(0);

        telemetry.addLine("Initialized — waiting for start");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {

            // ---- Drive Sequence ----
            moveForward(20, DRIVE_SPEED);
            sleep(500);


            moveBackward(20, DRIVE_SPEED);
            sleep(500);

            strafeRight(20, DRIVE_SPEED);
            sleep(500);

            strafeLeft(20, DRIVE_SPEED);
            sleep(500);

            // ---- Outtake Sequence ----
            runOuttake(2000); // run for 2 seconds

            // ---- Intake Sequence ----
            runIntake(2000); // run for 2 seconds

            telemetry.addLine("Auto Complete");
            telemetry.update();
        }
    }


    private void resetAndRunEncoders(DcMotor... motors) {
        for (DcMotor motor : motors) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    private void moveForward(double inches, double speed) {
        int move = (int) (inches * COUNTS_PER_INCH);
        setTargetPositionAll(move);
        runToPosition(speed);
    }

    private void moveBackward(double inches, double speed) {
        int move = (int) (inches * COUNTS_PER_INCH);
        setTargetPositionAll(-move);
        runToPosition(speed);
    }

    private void strafeRight(double inches, double speed) {
        int move = (int) (inches * COUNTS_PER_INCH);
        frontleft.setTargetPosition(frontleft.getCurrentPosition() + move);
        frontright.setTargetPosition(frontright.getCurrentPosition() - move);
        backleft.setTargetPosition(backleft.getCurrentPosition() - move);
        backright.setTargetPosition(backright.getCurrentPosition() + move);
        runToPosition(speed);
    }

    private void strafeLeft(double inches, double speed) {
        int move = (int) (inches * COUNTS_PER_INCH);
        frontleft.setTargetPosition(frontleft.getCurrentPosition() - move);
        frontright.setTargetPosition(frontright.getCurrentPosition() + move);
        backleft.setTargetPosition(backleft.getCurrentPosition() + move);
        backright.setTargetPosition(backright.getCurrentPosition() - move);
        runToPosition(speed);
    }

    private void setTargetPositionAll(int move) {
        frontleft.setTargetPosition(frontleft.getCurrentPosition() + move);
        frontright.setTargetPosition(frontright.getCurrentPosition() + move);
        backleft.setTargetPosition(backleft.getCurrentPosition() + move);
        backright.setTargetPosition(backright.getCurrentPosition() + move);
    }

    private void runToPosition(double speed) {
        frontleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backright.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontleft.setPower(speed);
        frontright.setPower(speed);
        backleft.setPower(speed);
        backright.setPower(speed);

        while (opModeIsActive() &&
                (frontleft.isBusy() && frontright.isBusy() && backleft.isBusy() && backright.isBusy())) {
            telemetry.addData("Moving...", "FL: %d FR: %d BL: %d BR: %d",
                    frontleft.getCurrentPosition(),
                    frontright.getCurrentPosition(),
                    backleft.getCurrentPosition(),
                    backright.getCurrentPosition());
            telemetry.update();
        }

        stopAllDriveMotors();
        resetAndRunEncoders(frontleft, frontright, backleft, backright);
        sleep(250);
    }

    private void stopAllDriveMotors() {
        frontleft.setPower(0);
        frontright.setPower(0);
        backleft.setPower(0);
        backright.setPower(0);
    }


    private void runIntake(long durationMs) {
        intake.setPower(1.0);
        telemetry.addLine("Intake running...");
        telemetry.addData("Intake Power", intake.getPower());
        telemetry.update();
        sleep(durationMs);
        intake.setPower(0);
        telemetry.addLine("Intake stopped");
        telemetry.update();
        sleep(250);
    }

    private void runOuttake(long durationMs) {
        outtakeleft.setPower(1.0);
        outtakeright.setPower(1.0);
        telemetry.addLine("Outtake running...");
        telemetry.addData("OuttakeLeft Power", outtakeleft.getPower());
        telemetry.addData("OuttakeRight Power", outtakeright.getPower());
        telemetry.update();
        sleep(durationMs);
        outtakeleft.setPower(0);
        outtakeright.setPower(0);
        telemetry.addLine("Outtake stopped");
        telemetry.update();
        sleep(250);
    }
}



/*package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Auto_Amulya", group = "Autonomous")
public class Auto_Amulya extends LinearOpMode {

    private DcMotor frontleft, frontright, backleft, backright;
    private DcMotor intake, outtakeleft, outtakeright;
    private ElapsedTime runtime = new ElapsedTime();

    static final double COUNTS_PER_MOTOR_REV = 383.6;   // goBILDA 20:1
    static final double DRIVE_GEAR_REDUCTION = 1.0;
    static final double WHEEL_DIAMETER_INCHES = 4.0;
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * Math.PI);
    static final double DRIVE_SPEED = 0.6;

    @Override
    public void runOpMode() {

        // ---- Drive Motors ----
        frontleft = hardwareMap.get(DcMotor.class, "frontleft");
        frontright = hardwareMap.get(DcMotor.class, "frontright");
        backleft = hardwareMap.get(DcMotor.class, "backleft");
        backright = hardwareMap.get(DcMotor.class, "backright");

        frontleft.setDirection(DcMotor.Direction.REVERSE);
        backleft.setDirection(DcMotor.Direction.REVERSE);
        frontright.setDirection(DcMotor.Direction.FORWARD);
        backright.setDirection(DcMotor.Direction.FORWARD);

        resetAndRunEncoders(frontleft, frontright, backleft, backright);

        // ---- Intake / Outtake Motors ----
        intake = hardwareMap.get(DcMotor.class, "intake");
        outtakeleft = hardwareMap.get(DcMotor.class, "outtakeleft");
        outtakeright = hardwareMap.get(DcMotor.class, "outtakeright");

        outtakeleft.setDirection(DcMotor.Direction.FORWARD);
        outtakeright.setDirection(DcMotor.Direction.REVERSE);

        intake.setPower(0);
        outtakeleft.setPower(0);
        outtakeright.setPower(0);

        telemetry.addLine("Initialized — waiting for start");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {

            // Forward 12 inches
            moveForward(20, DRIVE_SPEED);
            sleep(2000);

            moveForward(20, DRIVE_SPEED);
            sleep(2000);

            // Backward 12 inches
            moveBackward(20, DRIVE_SPEED);
            sleep(2000);

            // Strafe right 12 inches
            strafeRight(20, DRIVE_SPEED);
            sleep(2000);

            // Strafe left 15 inches (adjusted for mecanum precision)
            strafeLeft(20, DRIVE_SPEED);
            sleep(2000); // extra to allow settling

            // Run outtake for 2 seconds
            outtakeleft.setPower(1.0);
            outtakeright.setPower(1.0);
            telemetry.addLine("Running outtake...");
            telemetry.update();
            sleep(2000);
            outtakeleft.setPower(1.0);
            outtakeright.setPower(1.0);
            sleep(2000);

            // Run intake for 2 seconds
            intake.setPower(1.0);
            telemetry.addLine("Running intake...");
            telemetry.update();
            sleep(2000);
            intake.setPower(0);

            telemetry.addLine("Auto Complete!");
            telemetry.update();
        }
    }

    private void resetAndRunEncoders(DcMotor... motors) {
        for (DcMotor motor : motors) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    private void moveForward(double inches, double speed) {
        int move = (int)(inches * COUNTS_PER_INCH);
        setTargetPositionAll(move);
        runToPosition(speed);
    }

    private void moveBackward(double inches, double speed) {
        int move = (int)(inches * COUNTS_PER_INCH);
        setTargetPositionAll(-move);
        runToPosition(speed);
    }

    private void strafeRight(double inches, double speed) {
        int move = (int)(inches * COUNTS_PER_INCH);
        frontleft.setTargetPosition(frontleft.getCurrentPosition() + move);
        frontright.setTargetPosition(frontright.getCurrentPosition() - move);
        backleft.setTargetPosition(backleft.getCurrentPosition() - move);
        backright.setTargetPosition(backright.getCurrentPosition() + move);
        runToPosition(speed);
    }

    private void strafeLeft(double inches, double speed) {
        int move = (int)(inches * COUNTS_PER_INCH);
        frontleft.setTargetPosition(frontleft.getCurrentPosition() - move);
        frontright.setTargetPosition(frontright.getCurrentPosition() + move);
        backleft.setTargetPosition(backleft.getCurrentPosition() + move);
        backright.setTargetPosition(backright.getCurrentPosition() - move);
        runToPosition(speed);
    }

    private void setTargetPositionAll(int move) {
        frontleft.setTargetPosition(frontleft.getCurrentPosition() + move);
        frontright.setTargetPosition(frontright.getCurrentPosition() + move);
        backleft.setTargetPosition(backleft.getCurrentPosition() + move);
        backright.setTargetPosition(backright.getCurrentPosition() + move);
    }

    private void runToPosition(double speed) {
        frontleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backright.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontleft.setPower(speed);
        frontright.setPower(speed);
        backleft.setPower(speed);
        backright.setPower(speed);

        while (opModeIsActive() &&
                (frontleft.isBusy() && frontright.isBusy() && backleft.isBusy() && backright.isBusy())) {
            telemetry.addData("Moving...", "FL: %d FR: %d", frontleft.getCurrentPosition(), frontright.getCurrentPosition());
            telemetry.update();
        }

        stopAllDriveMotors();
        resetAndRunEncoders(frontleft, frontright, backleft, backright);
        sleep(250);
    }

    private void stopAllDriveMotors() {
        frontleft.setPower(0);
        frontright.setPower(0);
        backleft.setPower(0);
        backright.setPower(0);
    }
}

*/