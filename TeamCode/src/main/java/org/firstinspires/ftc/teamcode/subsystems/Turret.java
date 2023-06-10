package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Turret {
    private DcMotor turret;
    double multi = .5;
    //double rx = -gamepad.getRightX();
    static final double MOTOR_TICK_COUNT = 384.5;
    public Turret(HardwareMap hardwareMap) {

        turret = hardwareMap.dcMotor.get("turret");
       // turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
      // turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //turret.setTargetPosition(0);
         //turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }

    public void turnRight() {
        turret.setPower(1 * multi);

    }

    public void turnLeft() {

        turret.setPower(-1 * multi);
    }
    public void turnWithTrigger(double triggerX){
        turret.setPower(triggerX * multi);
    }
    public void presetTurretSide(){
        turret.setTargetPosition(0);
        turret.setPower(.1);
    }
    public void stopTurret(){
        turret.setPower(0);

    }
    public double getTurretPosition(){
        int currentPosition = turret.getCurrentPosition();
        return currentPosition;

    }
}

