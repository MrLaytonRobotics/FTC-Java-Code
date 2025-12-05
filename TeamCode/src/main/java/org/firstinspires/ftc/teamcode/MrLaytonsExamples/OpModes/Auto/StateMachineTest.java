package org.firstinspires.ftc.teamcode.MrLaytonsExamples.OpModes.Auto;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.MrLaytonsExamples.Mechanisms.Drivetrain;

public class StateMachineTest extends OpMode {

    Drivetrain drivetrain = new Drivetrain();

    enum State {
        WAIT_FOR_A,
        WAIT_FOR_B,
        WAIT_FOR_X,
        FINISHED
    }

    State state = State.WAIT_FOR_A;


    @Override
    public void init() {
        drivetrain.init(hardwareMap);
        state = State.WAIT_FOR_A;
    }

    @Override
    public void loop() {
        telemetry.addData("Cur State:", state);
        switch (state){
            case WAIT_FOR_A:
                telemetry.addLine("To exit state, press A");
                if (gamepad1.a) {
                    state = State.WAIT_FOR_B;
                }
                break;
            case WAIT_FOR_B:
                telemetry.addLine("To exit state, Press B");
                if (gamepad1.b) {
                    state = State.WAIT_FOR_X;
                }
                break;
            case WAIT_FOR_X:
                telemetry.addLine("To exit state, Press X");
                if (gamepad1.x) {
                    state = State.FINISHED;
                }
                break;
            default:
                telemetry.addLine("Auto state machine finished");
        }
    }





}
