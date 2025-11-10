package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.robot.Memory;

public class Drivetrain extends SubsystemBase {
    public static Follower follower;

    public Drivetrain(final HardwareMap hMap, boolean teleop, Pose start) {
        follower = Constants.createFollower(hMap);
        follower.setStartingPose(start);
        if (teleop) follower.startTeleOpDrive();
        follower.update();
    }

    @Override
    public void periodic() {
        Memory.robotHeading = follower.getHeading();
        Memory.robotAutoX = follower.getPose().getX();
        Memory.robotAutoY = follower.getPose().getY();
        follower.update();
    }
}
