package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.Gamepad;

public final class Spindexer {
	private final Servo spindexer;
	public Spindexer(HardwareMap hardwareMap) {
		spindexer = hardwareMap.get(Servo.class, "spindexer");
	}

	public void update() {
		// todo
	}

	public String getTelemetryData() {
		return String.format("Spindexer Position: %f", spindexer.getPosition());
	}
}