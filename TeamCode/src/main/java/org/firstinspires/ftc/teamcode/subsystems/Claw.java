package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotor;

public class Claw {
    private CRServoImplEx leftClaw, rightClaw;
    public Claw(CRServoImplEx leftClaw, CRServoImplEx rightClaw){
        this.leftClaw = leftClaw;
        this.rightClaw = rightClaw;
    }
}
