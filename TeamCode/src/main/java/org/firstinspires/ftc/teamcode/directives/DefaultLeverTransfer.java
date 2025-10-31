package org.firstinspires.ftc.teamcode.directives;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.stellarstructure.Trigger;
import org.firstinspires.ftc.teamcode.stellarstructure.conditions.GamepadButton;
import org.firstinspires.ftc.teamcode.stellarstructure.conditions.StatefulCondition;
import org.firstinspires.ftc.teamcode.stellarstructure.hardwaremapwrappers.StellarServo;
import org.firstinspires.ftc.teamcode.stellarstructure.runnables.DefaultDirective;
import org.firstinspires.ftc.teamcode.stellarstructure.runnables.Procedure;
import org.firstinspires.ftc.teamcode.stellarstructure.runnables.SetPosition;
import org.firstinspires.ftc.teamcode.stellarstructure.runnables.Sleep;
import org.firstinspires.ftc.teamcode.subsystems.LeverTransfer;

public class DefaultLeverTransfer extends DefaultDirective {
	//todo: implement starting conditions and directives and procedures
	public DefaultLeverTransfer(LeverTransfer leverTransfer, Gamepad gamepad, StellarServo leverTransferServo) {
		super(leverTransfer);

		addTrigger(new Trigger(
				new StatefulCondition(
						new GamepadButton(gamepad, GamepadButton.Button.DPAD_UP),
						StatefulCondition.Edge.RISING //On initial press
				),
				() -> {
					leverTransfer.setLeverPositionIsUp(true);
					leverTransfer.updateServoPosition();
				}
		));

		addTrigger(new Trigger(
				new StatefulCondition(
						new GamepadButton(gamepad, GamepadButton.Button.DPAD_DOWN),
						StatefulCondition.Edge.RISING //On initial press
				),
				() -> {
					leverTransfer.setLeverPositionIsUp(false);
					leverTransfer.updateServoPosition();
				}
		));

		addTrigger(new Trigger(
				new StatefulCondition(
						new GamepadButton(gamepad, GamepadButton.Button.DPAD_LEFT),
						StatefulCondition.Edge.RISING //On initial press
				),
				() -> {
					// up down up
					new Procedure(
						new SetPosition(leverTransferServo, LeverTransfer.LEVER_DOWN_POSITION, 0.01),
						new Sleep(0.03),
						new SetPosition(leverTransferServo, LeverTransfer.LEVER_UP_POSITION, 0.01),
						new Sleep(0.03),
						new SetPosition(leverTransferServo, LeverTransfer.LEVER_DOWN_POSITION, 0.01)
					).schedule();
				}
		));
	}
}