package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;



@TeleOp
public class HelloWorld extends OpMode {

    @Override
    public void init() {
        telemetry.addData(caption: "Hello", value: "World");
    }

    @Override
    public void loop() {
    }
}
