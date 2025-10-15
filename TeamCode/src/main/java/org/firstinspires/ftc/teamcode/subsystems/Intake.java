package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;


public final class Intake {
	private final DcMotorEx intake;
	private double intakeSpeed = 0;

	public Intake(HardwareMap hardwareMap) {
		intake = hardwareMap.get(DcMotorEx.class, "intake");
	}

	public void update(Gamepad gamepad) {
		if (gamepad.left_bumper) {
			intakeSpeed = -1.0;
		} else if (gamepad.right_bumper) {
			intakeSpeed = 1.0;
		}

		intake.setPower(intakeSpeed);
	}

	public String getTelemetryData() {
		return String.format("Intake Speed: %f", intakeSpeed);
	}
}