package org.firstinspires.ftc.teamcode.Utils;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;

@FunctionalInterface
public interface PathEnd {
    boolean finished(Pose2d error, PoseVelocity2d robotVel);
}
