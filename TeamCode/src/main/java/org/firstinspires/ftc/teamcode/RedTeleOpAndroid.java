package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.LLResult;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import java.util.List;


@TeleOp(name = "Red Teleop Android", group = "LionsSpark")
public class RedTeleOpAndroid extends LinearOpMode {

    // Declare motor and servo objects
    private DcMotor frontLeft = null;
    private DcMotor frontRight = null;
    private DcMotor backLeft = null;
    private DcMotor backRight = null;
    private DcMotorEx leftFlywheel;
    private DcMotorEx rightFlywheel;
    private DcMotor intake;
    private Limelight3A limelight;

    private Servo servo;
    private Servo alignCheckLight;

    // Setting our velocity targets. These values are in ticks per second!
    private static final double FULL_POWER = 1;
    private static final double bankVelocity = 1200;
    private static final int farVelocity = 1500;
    private static final int maxVelocity = 2000;

    // Limelight and distance calculation variables
    private static double targetOffsetAngle_Verticle;
    private static final double limelightMountAngleDegrees = 11;
    private static final double limelightLensHeightInches = 10.0;
    private static final double goalHeightInches = 29.5;
    private static double angleToGoalDegrees;
    private static double angleToGoalRadians;
    private static double distanceFromLimelightToGoalInches;
    private static double dist = 0;
    private static int learnedVelocity;
    private static double speed = 1400; //RPM
    private static double Tx;

    // State variables for toggles and modes
    private static boolean bPressedLast = false;
    private static boolean autoAim = false;
    private static boolean limelightIsValid = false;
    private static boolean aligning = false;

    // Enum for shooter state machine
    private enum ShootState {
        IDLE,
        PRELOAD,
        SHOOT
    }
    // Enum for alignment state machine
    private enum AlignState
    {
        STANDBY,
        ALIGNING,
        ALIGNED
    }
    private ShootState shootState = ShootState.IDLE;
    private AlignState alignState = AlignState.STANDBY;
    private ElapsedTime shootTimer = new ElapsedTime();


    private static boolean currentlyShooting = false;



    private boolean flywheelOn = false;
    private boolean right_bumperPressed = false;
    @Override
    public void runOpMode() {
        // Declares objects
        frontLeft  = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft   = hardwareMap.get(DcMotor.class, "backLeft");
        backRight  = hardwareMap.get(DcMotor.class, "backRight");
        leftFlywheel = hardwareMap.get(DcMotorEx.class, "leftFlywheel");
        rightFlywheel = hardwareMap.get(DcMotorEx.class, "rightFlywheel");
        intake = hardwareMap.get(DcMotor.class, "intake");
        limelight = hardwareMap.get(Limelight3A.class, "LimeLight");
        servo = hardwareMap.get(Servo.class, "servo");
        alignCheckLight = hardwareMap.get(Servo.class, "alignCheckLight");

        // Initialize toggle handlers
        Toggle flywheelToggle = new Toggle(false);
        Toggle limelightToggle = new Toggle(false);
        ToggleManager toggleManager = new ToggleManager();

        // Establishing the direction and mode for the motors
        rightFlywheel.setDirection(DcMotor.Direction.REVERSE);
        intake.setDirection(DcMotor.Direction.REVERSE);
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        //Ensures the servo is active and ready
        // --- MOTOR BEHAVIOR --- //
        // Drivetrain and Climber set to BRAKE mode to prevent coasting
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // Flywheels set to FLOAT mode
        leftFlywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightFlywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        // Initialize servo and light positions
        servo.setPosition(.4);
        alignCheckLight.setPosition(.288);
        alignCheckLight.setPosition(.277);
        // Set Limelight pipeline
        limelight.pipelineSwitch(0);

        //set encoders
        leftFlywheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFlywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFlywheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFlywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //defining auto aim switch



        ElapsedTime shootTime = new ElapsedTime();

        /*
         * Starts polling for data.  If you neglect to call start(), getLatestResult() will return null.
         */
        limelight.start();
        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive())
            {
                // Calling our methods while the OpMode is running
                autoAimToggle();
                limelightToggle.update(gamepad1.b);
                aligning = limelightToggle.getState();
                isAlignedLight(limelightToggle.getState());

                // Switch between manual and auto-aim driving
                if(!limelightToggle.getState())
                {
                    splitStickArcadeDrive();
                }
                else
                {
                    autoAimArcadeDrive(Tx);
                }
                telemetry.addData("Light Status", alignCheckLight.getPosition());


                //setFlywheelVelocity();
                //manualCoreHexAndServoControl();
                // Get and display Limelight status
                LLStatus status = limelight.getStatus();
                telemetry.addData("Name", "%s",status.getName());
                telemetry.addData("LL", "Temp: %.1fC, CPU: %.1f%%, FPS: %d",status.getTemp(), status.getCpu(),(int)status.getFps());
                telemetry.addData("Pipeline", "Index: %d, Type: %s", status.getPipelineIndex(), status.getPipelineType());
                telemetry.addData("Servo Position", servo.getPosition());
                //Height of april tag is 0.61 m

                //Toggle for the shooter
                servoAndIntakeToggle();
                shooterToggle();
                flywheelToggle.update(gamepad1.right_bumper);
                telemetry.addData("FlywheelStatus", flywheelOn);
                telemetry.addData("FlywheelToggleStatus", flywheelToggle.getState());

                // Control flywheel based on toggle state
                if(flywheelToggle.getState())
                {
                    shoot();
                }
                else
                {
                    stopFlywheels();
                }
                //Toggle for the servo claw and intake




                //Test for mapping power curve
                if(gamepad1.dpadUpWasPressed())
                {
                    speed += 50;
                }
                if(gamepad1.dpadDownWasPressed() && speed > 0)
                {
                    speed -= 50;
                }
                telemetry.addData("testSpeed", speed);
                LLResult result = limelight.getLatestResult();
                if(!result.isValid())
                {
                    speed = 1400;
                }
                // Process Limelight results if valid
                if (result.isValid())
                {
                    limelightIsValid = true;
                    // Access general information
                    Pose3D botpose = result.getBotpose();
                    double captureLatency = result.getCaptureLatency();
                    double targetingLatency = result.getTargetingLatency();
                    double parseLatency = result.getParseLatency();

                    // Calculate distance to goal
                    targetOffsetAngle_Verticle = (result.getTy());
                    angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Verticle;
                    angleToGoalRadians = angleToGoalDegrees * (Math.PI/180.0);
                    distanceFromLimelightToGoalInches = (goalHeightInches - limelightLensHeightInches) / Math.tan(angleToGoalRadians);
                    //attempt one for velocity calculation
                    dist = (1.0/Math.sqrt(result.getTa()));
                    learnedVelocity = (int)(dist * 1188);

                    // Display Limelight data in telemetry
                    telemetry.addData("Distance", 1/dist);
                    telemetry.addData("LL Latency", captureLatency + targetingLatency);
                    telemetry.addData("Parse Latency", parseLatency);
                    telemetry.addData("PythonOutput", java.util.Arrays.toString(result.getPythonOutput()));

                    telemetry.addData("tx", result.getTx());
                    telemetry.addData("txnc", result.getTxNC());
                    telemetry.addData("ty", result.getTy());
                    telemetry.addData("tync", result.getTyNC());
                    telemetry.addData("tarea", result.getTa());
                    telemetry.addData("angleToGoalDegrees", angleToGoalDegrees);
                    telemetry.addData("angleToGoalRadians", angleToGoalRadians);
                    telemetry.addData("distanceFromLimelightToGoalInches", distanceFromLimelightToGoalInches);
                    telemetry.addData("testSpeed", speed);
                    Tx = result.getTx();
                    // Calculate flywheel speed based on distance
                    speed = .2214*Math.pow(distanceFromLimelightToGoalInches,2) - 12.7553*distanceFromLimelightToGoalInches + 1216.8651;
                    telemetry.addData("Botpose", botpose.toString());

                    // Access barcode results
                    List<LLResultTypes.BarcodeResult> barcodeResults = result.getBarcodeResults();
                    for (LLResultTypes.BarcodeResult br : barcodeResults)
                    {
                        telemetry.addData("Barcode", "Data: %s", br.getData());
                    }

                    // Access classifier results
                    List<LLResultTypes.ClassifierResult> classifierResults = result.getClassifierResults();
                    for (LLResultTypes.ClassifierResult cr : classifierResults)
                    {
                        telemetry.addData("Classifier", "Class: %s, Confidence: %.2f", cr.getClassName(), cr.getConfidence());
                    }

                    // Access detector results
                    List<LLResultTypes.DetectorResult> detectorResults = result.getDetectorResults();
                    for (LLResultTypes.DetectorResult dr : detectorResults)
                    {
                        telemetry.addData("Detector", "Class: %s, Area: %.2f", dr.getClassName(), dr.getTargetArea());
                    }

                    // Access fiducial results
                    List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();

                    for (LLResultTypes.FiducialResult fr : fiducialResults)
                    {
                        telemetry.addData("Fiducial", "ID: %d, Family: %s, X: %.2f, Y: %.2f", fr.getFiducialId(), fr.getFamily(), fr.getTargetXDegrees(), fr.getTargetYDegrees());
                    }

                    // Access color results
                    List<LLResultTypes.ColorResult> colorResults = result.getColorResults();
                    for (LLResultTypes.ColorResult cr : colorResults)
                    {
                        telemetry.addData("Color", "X: %.2f, Y: %.2f", cr.getTargetXDegrees(), cr.getTargetYDegrees());
                    }
                }
                else
                {
                    telemetry.addData("Limelight", "No data available");
                    limelightIsValid = false;
                }


                // Display flywheel and velocity data
                telemetry.addData("Learned Velocity", learnedVelocity);
                telemetry.addData("Left Flywheel Velocity", ((DcMotorEx) leftFlywheel).getVelocity());
                telemetry.addData("Right Flywheel Velocity", ((DcMotorEx) rightFlywheel).getVelocity());
                telemetry.addData("Left Flywheel Power", leftFlywheel.getPower());
                telemetry.addData("Right Flywheel Power", rightFlywheel.getPower());
                telemetry.update();
            }
            limelight.stop();
        }
    }

    /**
     * Controls for the drivetrain. The robot uses a split stick stlye arcade drive.
     * Forward and back is on the left stick. Turning is on the right stick.
     */
    private void splitStickArcadeDrive() {
        //----------------//
        // MECANUM DRIVE  //
        //----------------//
        double rx = gamepad1.left_stick_y + gamepad1.right_stick_y;
        double x =  -gamepad1.left_stick_x;
        double y = gamepad1.right_stick_x;
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y + x - rx) / denominator;
        double backRightPower = (y - x - rx) / denominator;

        frontLeft.setPower(frontLeftPower);
        backLeft.setPower(backLeftPower);
        frontRight.setPower(frontRightPower);
        backRight.setPower(backRightPower);

    }

    /**
     * Controls the drivetrain with auto-aim functionality.
     * @param Tx The horizontal offset from the Limelight target.
     */
    private void autoAimArcadeDrive(double Tx) {
        double y;
        //----------------//
        // AUTOAIM DRIVE  //
        //----------------//
        if(limelightIsValid == false){
            y = gamepad1.right_stick_x;
        }else{
            if(Math.abs(Tx) < 3){
                y = 0;
            }else{
                y = Tx/40;
            }
        }




        double rx = gamepad1.left_stick_y + gamepad1.right_stick_y;
        double x =  -gamepad1.left_stick_x;

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y + x - rx) / denominator;
        double backRightPower = (y - x - rx) / denominator;

        frontLeft.setPower(frontLeftPower);
        backLeft.setPower(backLeftPower);
        frontRight.setPower(frontRightPower);
        backRight.setPower(backRightPower);

    }

    /**
     * Manual control for the Core Hex powered feeder and the agitator servo in the hopper
     */

    //Toggle for servo
    private void servoAndIntakeToggle() {

        // Intake control (manual overrides)
        if (gamepad1.left_trigger > 0.1) {
            intake.setPower(1);
            return; // Skip shooting logic while intaking
        }
        if (gamepad1.left_bumper) {
            intake.setPower(-1);
            return;
        }

        // Shooting state machine
        switch (shootState) {

            case IDLE:
                if (gamepad1.right_trigger > 0.1) {
                    // Trigger pressed → start preload phase
                    shootState = ShootState.PRELOAD;
                    shootTimer.reset();
                }
                else {
                    // Default idle state
                    intake.setPower(0);
                    servo.setPosition(0.3);
                }
                break;

            case PRELOAD:
                // Run intake backwards briefly to "pull back" the ring
                intake.setPower(-1);

                // After 0.15 seconds, go to shooting phase
                if (shootTimer.seconds() > 0.15) {
                    shootState = ShootState.SHOOT;
                    servo.setPosition(0); // Fire
                }
                break;

            case SHOOT:
                if (gamepad1.right_trigger > 0.1) {
                    // Keep shooting while the trigger is held
                    intake.setPower(1);
                } else {
                    // Trigger released → stop everything
                    intake.setPower(0);
                    servo.setPosition(0.3);
                    shootState = ShootState.IDLE;
                }
                break;
        }
    }

    /**
     * Toggles the auto-aim feature.
     */
    private void autoAimToggle() {
        boolean bPressedNow = gamepad1.b;
        if(bPressedNow && !bPressedLast)
        {
            autoAim = !autoAim;

        }
        bPressedLast = bPressedNow;
    }

    /**
     * Toggles the shooter flywheel on and off.
     */
    private void shooterToggle() {
        if (gamepad1.right_bumper && !right_bumperPressed)
        {
            flywheelOn = !flywheelOn;
        }
        right_bumperPressed = gamepad1.right_bumper;

    }


    //Flywheel Code
    /**
     * This if/else statement contains the controls for the flywheel, both manual and auto.
     * Circle and Square will spin up ONLY the flywheel to the target velocity set.
     * The bumpers will activate the flywheel, Core Hex feeder, and servo to cycle a series of balls.
     */


    /**
     * Ramps up the flywheel to the target speed.
     */
    private void shoot()
    {
        double currentVelocity = leftFlywheel.getVelocity();
        if(currentVelocity < speed)
        {
            leftFlywheel.setPower(FULL_POWER);
            rightFlywheel.setPower(FULL_POWER);


        }
        else if(currentVelocity >= (speed -100) || currentVelocity >= (speed + 100))
        {
            leftFlywheel.setVelocity(speed);
            rightFlywheel.setVelocity(speed);
        }
        else
        {
            leftFlywheel.setPower(0);
            rightFlywheel.setPower(0);
        }
        //servo.setPower(-1);

    }

    /**
     * The far power velocity is intended for launching balls a few feet from the goal. It may require adjusting the deflector.
     * When running this function, the flywheel will spin up and the Core Hex will wait before balls can be fed.
     * The servo will spin until the bumper is released.
     */

    /**
     * Stops the flywheels.
     */
    private void stopFlywheels() {
        ((DcMotorEx) leftFlywheel).setVelocity(0);
        ((DcMotorEx) rightFlywheel).setVelocity(0);

    }

    /**
     * Controls the alignment indicator light based on the alignment status.
     * @param toggled Whether the alignment mode is toggled on.
     */
    private void isAlignedLight(boolean toggled)
    {
        switch(alignState) {
            case STANDBY:
                alignCheckLight.setPosition(.288);
                if (toggled) {
                    alignState = AlignState.ALIGNING;
                }
                break;
            case ALIGNING:
                alignCheckLight.setPosition(.333);
                if ((Math.abs(Tx) < 3) && (speed >= leftFlywheel.getVelocity() - 100 && speed <= leftFlywheel.getVelocity() + 100)) {
                    alignState = AlignState.ALIGNED;
                }
                if (!toggled)
                {
                    alignState = AlignState.STANDBY;
                }
                break;

            case ALIGNED:
                alignCheckLight.setPosition(.5);
                if(!((Math.abs(Tx) < 3) && (speed >= leftFlywheel.getVelocity() - 100 && speed <= leftFlywheel.getVelocity() + 100)))
                {
                    alignState = AlignState.ALIGNING;
                }
                if (!toggled)
                {
                    alignState = AlignState.STANDBY;
                }
                break;
        }
    }

}
