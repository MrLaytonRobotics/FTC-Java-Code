package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.directives.DefaultLeverTransfer;
import org.firstinspires.ftc.teamcode.stellarstructure.hardwaremapwrappers.StellarServo;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.stellarstructure.Subsystem;
import org.firstinspires.ftc.teamcode.stellarstructure.runnables.Directive;
import org.firstinspires.ftc.teamcode.stellarstructure.runnables.MoveTo;
import org.firstinspires.ftc.teamcode.stellarstructure.runnables.SetPosition;

public final class LeverTransfer extends Subsystem {
	private static final LeverTransfer leverTransfer = new LeverTransfer();

	public static LeverTransfer getInstance() {
		return leverTransfer;
	}

	private LeverTransfer() {}

	private StellarServo leverTransferServo;

	public final static double LEVER_DOWN_POSITION = 0.28;
	public final static double LEVER_UP_POSITION = 0.0;

	private boolean isLeverTargetUp = false;

	@Override
	public void init(HardwareMap hardwareMap) {
		leverTransferServo = new StellarServo(hardwareMap, "leverTransfer");
	}

	@Override
	public void setGamepads(Gamepad gamepad1, Gamepad gamepad2) {
		setDefaultDirective(new DefaultLeverTransfer(this, gamepad1, leverTransferServo));
	}

	@Override
	public void update() {}

	public void setLeverPositionIsUp(boolean isUpPosition) {
		isLeverTargetUp = isUpPosition;
	}

	public void toggleLeverPosition() {
		isLeverTargetUp = !isLeverTargetUp;
	}

	public void updateServoPosition() {
		//todo: fix directive spam
		new SetPosition(
				leverTransferServo,
				isLeverTargetUp ? LEVER_UP_POSITION : LEVER_DOWN_POSITION,
				0.01
		).setStartingConditions(
				() -> !Spindexer.getInstance().getIsIntakePosition()
		).schedule();
	}

	public boolean getIsLeverUp() {
		return !isLeverTargetUp;
	}

	@Override
	public String getTelemetryData() {
		return String.format("Lever Up Position: %f\nLever Is Up: %b", LEVER_UP_POSITION, isLeverTargetUp);
	}
}