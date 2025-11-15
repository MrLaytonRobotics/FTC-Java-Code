package org.firstinspires.ftc.teamcode.Teleop.Rev;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

@TeleOp
public class OldRevTeleOp extends OpMode {
    private DcMotor rightFront;
    private DcMotor rightRear;
    private DcMotor leftFront;
    private DcMotor leftRear;
    //D

    @Override
    //runs before anything
    public void init() {
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightRear = hardwareMap.get(DcMotor.class, "rightRear");
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftRear = hardwareMap.get(DcMotor.class, "leftRear");

    //change this when working!!!!!
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        rightRear.setDirection(DcMotor.Direction.FORWARD);
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        leftRear.setDirection(DcMotor.Direction.FORWARD);
    }
    //runs after init but before started.
    @Override
    public void init_loop() {

    }
    //runs when hits start button
    @Override
    public void start() {

    }
    //runs after hits start button
    @Override
    public void loop() {
        Movement();
    }

    @Override
    public void stop() {

    }
    public void Movement() {
        float gamePadY = -gamepad1.left_stick_y;
        float gamePadX = gamepad1.right_stick_x;

        //Range.clip clamps the value
        rightFront.setPower(Range.clip(gamePadY - gamePadX, -1, 1));
        rightRear.setPower(Range.clip(gamePadY - gamePadX, -1, 1));
        leftFront.setPower(Range.clip(gamePadY + gamePadX, -1, 1));
        leftRear.setPower(Range.clip(gamePadY + gamePadX, -1, 1));
    }
}