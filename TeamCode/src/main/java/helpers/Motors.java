package helpers;

import com.qualcomm.robotcore.hardware.DcMotorEx;

public class Motors {
    public void SpinupDcMotor(DcMotorEx motor, float speed, float velocity) {
        if (Math.abs(motor.getVelocity()) != speed){
            motor.setVelocity(velocity);
        } else {
            //donothong
        }
    }

}
