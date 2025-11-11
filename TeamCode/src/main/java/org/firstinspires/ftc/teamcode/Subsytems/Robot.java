package org.firstinspires.ftc.teamcode.Subsytems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public class Robot {
    public MecanumDrive mecanum;
    public Intake intake;
    private final OpMode opMode;
    private Turret turret;

    public Robot(OpMode opMode)
    {
        this.opMode=opMode;

        //this.mecanum=new PinpointDrive(opMode.hardwareMap, new Pose2d(0,0,0));
        this.turret=new Turret(opMode);
    }
}
