package org.firstinspires.ftc.teamcode;

//import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
//import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Gamepad;
//import com.qualcomm.robotcore.hardware.CRServo;
//import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;


public class intakeoutake_Amulya {

    // INSTANTIATE MOTORS AND SERVOS
    private DcMotor intake;
    //private Servo claw;

    private boolean clawOpen = true;
    private boolean lastBump = false;

    public void init(HardwareMap hardwareMap) {
        intake = hardwareMap.get(DcMotor.class, "intake");
        //claw = hardwareMap.get(DcMotor.class, "claw");
        intake.setDirection(DcMotor.Direction.REVERSE);
        /* clawOpen = false; */
        lastBump = false;
    }

  /*  public void GAMEPAD_INPUT_TOGGLE(Gamepad gamepad2, Telemetry telemetry) {
        // Toggle claw position when right_bumper is pressed
        telemetry.addData("lastBump - preOp", lastBump);
        if (gamepad2.right_bumper && !lastBump) {
            this.clawOpen = !this.clawOpen;
            telemetry.addData("clawOpen", clawOpen);
            if (this.clawOpen) {
                //OPEN
                claw.setPosition(0.01);
            } else {
                //CLOSE
                claw.setPosition(1.1);
            }
        }
        this.lastBump = gamepad2.right_bumper;
        telemetry.addData("lastBump - postOp", lastBump);
        telemetry.update();
    }
*/
    public void GAMEPAD_INTAKE(Gamepad gamepad2, Telemetry telemetry) {
        // Control intake motor with triggers
        if (gamepad2.right_trigger > 0.1) {
            intake.setPower(1.0);
        } else if (gamepad2.left_trigger > 0.1) {
            intake.setPower(-1.0);
        } else {
            intake.setPower(0);
        }
    }

}
