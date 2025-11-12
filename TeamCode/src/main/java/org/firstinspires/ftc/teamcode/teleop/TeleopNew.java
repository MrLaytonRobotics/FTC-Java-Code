package org.firstinspires.ftc.teamcode.teleop;


import com.acmerobotics.dashboard.config.Config;
import com.bylazar.gamepad.Gamepad;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.button.GamepadButton;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.seattlesolvers.solverslib.util.TelemetryData;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.robot.Memory;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

@Config
@TeleOp
public class TeleopNew extends CommandOpMode {
    Follower follower;
    TelemetryData telemetryData = new TelemetryData(telemetry);
    private GamepadEx gamepad;
    private Intake intake;
    private Shooter shooter;
    public static double shooterX, shooterY;

    @Override
    public void initialize() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(Memory.robotPose);
        super.reset();

        follower.startTeleopDrive();
        gamepad = new GamepadEx(gamepad1);
        intake = new Intake(hardwareMap);

        if (Memory.allianceRed) {
            shooterX = 144;
            shooterY = 144;
        } else {
            shooterX = 0;
            shooterY = 144;
        }

        shooter = new Shooter(hardwareMap, () -> follower.getPose(), shooterX, shooterY);

        gamepad.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(
                intake.collect()
        );

        gamepad.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(
                intake.reverse()
        );

        gamepad.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenReleased(
                intake.stop()
        );

        gamepad.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(
                new ParallelCommandGroup(
                    intake.collect(),
                    intake.open()
                )
        );

        gamepad.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenReleased(
                new ParallelCommandGroup(
                        intake.stop(),
                        intake.close()
                )
        );

        gamepad.getGamepadButton(GamepadKeys.Button.A).whenPressed(
                shooter.flywheel(true)
        );

        gamepad.getGamepadButton(GamepadKeys.Button.B).whenPressed(
                shooter.flywheel(false)
        );
    }

    @Override
    public void run() {
        super.run();

        follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, true);
        follower.update();

        telemetryData.addData("X", follower.getPose().getX());
        telemetryData.addData("Y", follower.getPose().getY());
        telemetryData.addData("Heading", Math.toDegrees(follower.getPose().getHeading()));
        telemetryData.update();
    }
}