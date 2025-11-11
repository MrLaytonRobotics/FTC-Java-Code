package org.firstinspires.ftc.teamcode.Subsytems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
@TeleOp
public class Mecanum3 extends LinearOpMode {
    public DcMotor motorOne,motorTwo,motorThree;
    @Override
    public void runOpMode() throws InterruptedException {
        motorOne=hardwareMap.get(DcMotor.class,"motorOne");
        motorTwo=hardwareMap.get(DcMotor.class,"motorTwo");
        motorThree=hardwareMap.get(DcMotor.class,"motorThree");
        motorOne.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorThree.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorTwo.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();
        while (opModeIsActive()) {
            double leftStickXValue = gamepad1.left_stick_x;
            double leftStickYValue= gamepad1.left_stick_y;
            double rightStickXValue = gamepad1.right_stick_x;

            double motorOnePower=-leftStickXValue;
            double motorTwoPower=leftStickXValue /2;
            double motorThreePower = leftStickXValue / 2;

            leftStickYValue =leftStickYValue*(Math.sqrt(3) / 2);
            motorTwoPower += (leftStickYValue);
            motorThreePower += leftStickYValue;

            motorOnePower += rightStickXValue;
            motorTwoPower+=rightStickXValue;
            motorThreePower += rightStickXValue;
        if (Math.abs(motorOnePower) > 1 || Math.abs(motorTwoPower) > 1 ||  Math. abs(motorThreePower) > 1) {
            double maxPower = findAbsoluteMax(motorOnePower, motorTwoPower, motorThreePower);
            motorOnePower /= maxPower;
            motorTwoPower /= maxPower;
            motorThreePower /= maxPower;
        }
        motorOne.setPower (motorOnePower);
        motorTwo.setPower (motorTwoPower);
        motorThree.setPower (motorThreePower);


        }

    }

    double findAbsoluteMax(double power1, double power2, double power3) {
        double max;
        max = Math.max(Math.abs(power1), Math.abs(power2));  max = Math.max(max, Math.abs(power3));
        return max;}
}
