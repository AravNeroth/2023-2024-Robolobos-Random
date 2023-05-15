package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class Turrent {
    private DcMotor zTurrent;
    private DcMotor yTurrent;

    public Turrent(DcMotor zTurrent, DcMotor yTurrent){
        this.zTurrent = zTurrent;
        this.yTurrent = yTurrent;

    }
}
