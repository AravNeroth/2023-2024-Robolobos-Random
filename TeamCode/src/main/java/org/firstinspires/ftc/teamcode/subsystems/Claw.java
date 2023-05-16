package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotor;

public class Claw {
    CRServoImplEx leftClaw = hardwareMap.get(CRServoImplEx.class, "leftClaw");
    CRServoImplEx rightClaw = hardwareMap.get(CRServoImplEx.class, "rightClaw");

}
