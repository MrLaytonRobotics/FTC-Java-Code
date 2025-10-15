package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.Gamepad;

public final class LeverTransfer {
	private final Servo leverTransfer;
	private final double leverDownPosition;
	private double leverUpPosition;
	private boolean leverTargetIsUpPosition = false;
	private boolean dpadUpPressedLastFrame, dpadDownPressedLastFrame = false;

    public LeverTransfer(double leverDownPosition, double startingLeverUpPosition, HardwareMap hardwareMap) {
		this.leverDownPosition = leverDownPosition;
		this.leverUpPosition = startingLeverUpPosition;
		leverTransfer = hardwareMap.get(Servo.class, "leverTransfer");
	}

	public void update(Gamepad gamepad) {
		if (gamepad.dpad_up && !dpadUpPressedLastFrame) {
			leverUpPosition = Math.min(leverUpPosition + 0.05, 1.0);
			dpadUpPressedLastFrame = true;
		} else {
			dpadUpPressedLastFrame = false;
		}

		if (gamepad.dpad_down && !dpadDownPressedLastFrame) {
			leverUpPosition = Math.max(leverUpPosition - 0.05, 0.05);
			dpadDownPressedLastFrame = true;
		} else {
			dpadDownPressedLastFrame = false;
		}

		if (gamepad.dpad_left) {
			leverTargetIsUpPosition = !leverTargetIsUpPosition;
		}

		leverTransfer.setPosition(leverTargetIsUpPosition ? leverUpPosition : leverDownPosition);
	}

	public String getTelemetryData() {
		return String.format("Lever Up Position: %f\nLever Is Up: %f", leverUpPosition, leverTargetIsUpPosition);
	}
}