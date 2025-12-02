package org.firstinspires.ftc.teamcode.MrLaytonsExamples.OpModes.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.MrLaytonsExamples.Mechanisms.Drivetrain;

@Autonomous
public class BlueAlliance extends OpMode {

    Drivetrain drivetrain = new Drivetrain();

    @Override
    public void init() {

        drivetrain.init(hardwareMap);

    }

    @Override
    public void loop() {

        drivetrain.autoDrive(0.5,10,10,5000);
        drivetrain.autoDrive(0.5,-8,8, 5000);

    }


}
