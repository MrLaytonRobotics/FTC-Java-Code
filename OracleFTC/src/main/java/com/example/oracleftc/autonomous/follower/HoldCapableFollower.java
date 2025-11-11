package com.example.oracleftc.autonomous.follower;

import com.smartcluster.oracleftc.commands.Command;
import com.smartcluster.oracleftc.math.Pose2d;

public interface HoldCapableFollower {
    Command hold(Pose2d pose);
}
