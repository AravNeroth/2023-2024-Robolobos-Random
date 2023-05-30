package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.arcrobotics.ftclib.hardware.RevIMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Turrent {
    DcMotor turrent;
    public Turrent(HardwareMap hardwareMap) {
        turrent = hardwareMap.dcMotor.get("turrent");
    }

    public void turnRight() {
        turrent.setPower(1);
    }

    public void turnLeft() {
        turrent.setPower(-1);
    }
}
