package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Turret {
    DcMotor turret;
    public Turret(HardwareMap hardwareMap) {
        turret = hardwareMap.dcMotor.get("turret");
    }

    public void turnRight() {
        turret.setPower(1);
    }

    public void turnLeft() {
        turret.setPower(-1);
    }
}
