package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class Turrent {
    private DcMotor turrent;

    public Turrent(DcMotor turrent){
        this.turrent = turrent;
    }

    public void turnRight() {
        turrent.setPower(1);
    }

    public void turnLeft() {
        turrent.setPower(-1);
    }
}
