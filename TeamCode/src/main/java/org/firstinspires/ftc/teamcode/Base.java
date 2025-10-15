package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.subsystems.Drivebase;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.LeverTransfer;
import org.firstinspires.ftc.teamcode.subsystems.Spindexer;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp(name = "Base TARS", group = "Robot")
public abstract class Base extends LinearOpMode {
	@Override
	public void runOpMode() {
		Drivebase drivebase = new Drivebase(1.00, 1.00, hardwareMap);
		Intake intake = new Intake(hardwareMap);
		LeverTransfer leverTransfer = new LeverTransfer(0.00, 0.60, hardwareMap);
		Spindexer spindexer = new Spindexer(hardwareMap);

		waitForStart();

		if (isStopRequested()) return;

		while (opModeIsActive()) {
			drivebase.update(gamepad1);
			intake.update(gamepad1);
			leverTransfer.update(gamepad1);
			spindexer.update();

			telemetry.addLine(drivebase.getTelemetryData());
			telemetry.addLine(intake.getTelemetryData());
			telemetry.addLine(leverTransfer.getTelemetryData());
			telemetry.addLine(spindexer.getTelemetryData());

			telemetry.update();
		}
	}
}