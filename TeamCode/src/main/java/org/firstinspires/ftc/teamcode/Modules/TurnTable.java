package org.firstinspires.ftc.teamcode.Modules;

import org.firstinspires.ftc.teamcode.riptideUtil;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.ArrayList;

public class TurnTable {
    Camera camera;
    Servo turntable_servo;
    DcMotor turntable_motor;
    PIDController tablePID;

    double tableKP = 0, tableKI = 0, tableKD = 0; // needs tuning

    // This is the name of the tag which the turn table will be following
    // the goal_tag can be either "Red Goal" or "Blue Goal"
    String goal_tag;

    public TurnTable(HardwareMap hardwareMap, String goal_tag) {
        this.camera = new Camera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        this.turntable_servo = hardwareMap.servo.get("turntables"); // turn table servo
        this.turntable_motor = hardwareMap.dcMotor.get("turntablem"); // turn table motor
        this.tablePID = new PIDController(tableKP, tableKI, tableKD); // gotta tune
        this.goal_tag = goal_tag;
    }

    public void setPosition(double pos) {
        /*
         * This would be relative to the robot,
         * I don't know if we are using a servo or a motor so the
         * code might be a bit different
         */
        // for the servo
        turntable_servo.setPosition(tablePID.calculate(turntable_servo.getPosition(), pos));

        // for the motor
        turntable_motor.setPower(tablePID.calculate(0, pos));
    }

    public void followTag() {
        /*
         * This is going to be constantly called to update the servo position or
         * motor power. It looks for the april tag and finds the position needed
         * so that the turn table faces the tag.
         */
        ArrayList<AprilTagDetection> detections = camera.getTagDetections();

        for (AprilTagDetection detection : detections) {
            if (detection.metadata.name.equals(goal_tag)) {
                double horiz_angle = camera.getTagHorizonalAngle(detection);
                setPosition(horiz_angle * riptideUtil.MOTOR_POS_CONST * 2 / riptideUtil.CAMERA_WIDTH);
            }
        }
    }
}
