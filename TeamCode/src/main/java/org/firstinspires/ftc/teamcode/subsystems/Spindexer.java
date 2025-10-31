package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.directives.DefaultSpindexer;
import org.firstinspires.ftc.teamcode.stellarstructure.Subsystem;

import org.firstinspires.ftc.teamcode.stellarstructure.hardwaremapwrappers.StellarServo;

public final class Spindexer extends Subsystem {
	private static final Spindexer spindexer = new Spindexer();

	public static Spindexer getInstance() {
		return spindexer;
	}

	private Spindexer() {}

	private final static double DEGREES_TO_SERVO = 1.0 / 320.0;
	private int selectedSegment = 0;
	private boolean isIntakePosition = true;
	private final static double SPINDEXER_OFFSET = 20.0;
	private final static double[] INTAKE_DEGREE_POSITIONS = {0.0, 240.0, 120.0};
	private final static double[] TRANSFER_DEGREE_POSITIONS = {180.0, 60.0, 300.0};

	private StellarServo spindexerServo;
	private DigitalChannel beamBreak;
	private ColorSensor colorSensor;

	@Override
	public void init(HardwareMap hardwareMap) {
		spindexerServo = new StellarServo(hardwareMap, "spindexer");
		beamBreak = hardwareMap.get(DigitalChannel.class, "beamBreak"); //unused
		beamBreak.setMode(DigitalChannel.Mode.INPUT);

		colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");
	}

	@Override
	public void setGamepads(Gamepad gamepad1, Gamepad gamepad2) {
		setDefaultDirective(new DefaultSpindexer(this, gamepad1));
	}

	@Override
	public void update() {}

	public void setSelectedSegment(int selectedSegment) {
		this.selectedSegment = selectedSegment;
	}

	public void toggleIsIntakePosition() {
		isIntakePosition = !isIntakePosition;
	}

	public void updateServoPosition() {
		spindexerServo.setPosition(
				((isIntakePosition ?
						INTAKE_DEGREE_POSITIONS[selectedSegment] :
						TRANSFER_DEGREE_POSITIONS[selectedSegment]
				) + SPINDEXER_OFFSET) * DEGREES_TO_SERVO
		);
	}

	public boolean getIsIntakePosition() {
		return isIntakePosition;
	}

	public int getSelectedSegment() {
		return selectedSegment;
	}

	@Override
	public String getTelemetryData() {
		return String.format(
				"selectedSegment: %d\n" +
				"isIntakePosition: %b\n" +
				"beamBreak: %b\n" +
				"colorSensorRGB: %d, %d, %d",
				selectedSegment, isIntakePosition, beamBreak.getState(),
				colorSensor.red(), colorSensor.green(), colorSensor.blue()
		);
	}
}