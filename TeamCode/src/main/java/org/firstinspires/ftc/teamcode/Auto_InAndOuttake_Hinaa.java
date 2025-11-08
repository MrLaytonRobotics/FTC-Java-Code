package org.firstinspires.ftc.teamcode;

//import static org.firstinspires.ftc.robotcore.external.telemetry;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "Intake and Outtake", group = "Autonomous")
public class Auto_InAndOuttake_Hinaa extends LinearOpMode {

    private DcMotor intake;
    private DcMotor outtakeleft;
    private DcMotor outtakeright;

    @Override
    public void runOpMode() {
        // Map hardware
        intake = hardwareMap.get(DcMotor.class, "intake");
        outtakeleft = hardwareMap.get(DcMotor.class, "outtakeleft");
        outtakeright = hardwareMap.get(DcMotor.class, "outtakeright");

        // Make the two outtakes spin opposite ways
        outtakeleft.setDirection(DcMotor.Direction.FORWARD);
        outtakeright.setDirection(DcMotor.Direction.REVERSE);

        // Set all powers to zero initially
        intake.setPower(0);
        outtakeleft.setPower(0);
        outtakeright.setPower(0);

        telemetry.addLine("Initialized — waiting for start");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            // Step 1: Run intake to pick up pixel
            intake.setPower(1.0);
            telemetry.addLine("Running intake...");
            telemetry.update();
            sleep(2000);  // run intake for 2 seconds

            // Step 2: Stop intake
            intake.setPower(0);
            sleep(500);

            // Step 3: Run outtake to shoot pixel
            outtakeleft.setPower(1.0);
            outtakeright.setPower(1.0);
            telemetry.addLine("Shooting artifact...");
            telemetry.update();
            sleep(3000);  // spin shooters for 3 seconds

            // Step 4: Stop all motors
            intake.setPower(0);
            outtakeleft.setPower(0);
            outtakeright.setPower(0);
            telemetry.addLine("Done");
            telemetry.update();
        }
    }
}
