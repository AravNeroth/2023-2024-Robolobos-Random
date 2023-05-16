package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class Turrent {
    DcMotor turrentRot = hardwareMap.dcMotor.get("turrent");
    public void turnRight() {
        turrentRot.setPower(1);
    }

    public void turnLeft() {
        turrentRot.setPower(-1);
    }
}
