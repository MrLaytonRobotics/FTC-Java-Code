package org.firstinspires.ftc.teamcode.MrLaytonsExamples.RobotControl;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;

public class ArcadeDrive {

    double throttle = -gamepad1.left_stick_y;
    double rotation = gamepad1.left_stick_x;

}
