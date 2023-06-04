package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Turret {
    DcMotor turret;
    //double rx = -gamepad.getRightX();
    public Turret(HardwareMap hardwareMap) {

        turret = hardwareMap.dcMotor.get("turret");
    }

    public void turnRight() {
        turret.setPower(1);

    }

    public void turnLeft() {
       turret.setPower(-1);
    }
    public void stopTurret(){
        turret.setPower(0);
    }
}
