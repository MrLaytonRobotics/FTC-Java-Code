package org.firstinspires.ftc.teamcode.pedroPathing;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
@Configurable
public class Constants {
    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("rf") // 0
            .rightRearMotorName("rb") // 1
            .leftRearMotorName("lb") // 2
            .leftFrontMotorName("lf") // 3
            .leftFrontMotorDirection(DcMotorEx.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorEx.Direction.FORWARD)
            .leftRearMotorDirection(DcMotorEx.Direction.REVERSE)
            .rightRearMotorDirection(DcMotorEx.Direction.FORWARD)
            .xVelocity(75)
            .yVelocity(63.94);
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(9.98)
            .forwardZeroPowerAcceleration(-43.6)
            .lateralZeroPowerAcceleration(-73.1)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.1, 0, 0.01, 0.01))
            .headingPIDFCoefficients(new PIDFCoefficients(0.96, 0, 0.03, 0.01))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.013,0.0,0.0004,0.6,0.04))
            .centripetalScaling(0.00054);
    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(2.25)
            .strafePodX(-6)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);
    public static PathConstraints pathConstraints2 = new PathConstraints(0.995, 100, 1, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pinpointLocalizer(localizerConstants)
                .pathConstraints(pathConstraints2)
                .mecanumDrivetrain(driveConstants)
                .build();
    }
}