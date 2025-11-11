package com.example.oracleftc.autonomous.localization;

import com.smartcluster.oracleftc.math.Pose2d;
import com.smartcluster.oracleftc.math.Pose2dDual;
import com.smartcluster.oracleftc.math.Time;

public interface Localizer {
    void setPose(Pose2d pose);
    Pose2dDual<Time> getPose();
    void update();
}
