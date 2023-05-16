package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotor;

public class Arm {
    private DcMotor arm;
    DcMotor turrentArm = hardwareMap.dcMotor.get("arm");
    CRServoImplEx slide = hardwareMap.get(CRServoImplEx.class, "slide");
    public Arm(){

    }
}
