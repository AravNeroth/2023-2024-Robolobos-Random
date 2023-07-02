package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import java.lang.Math;
import com.arcrobotics.ftclib.controller.PIDController;

public class Turret {
    private final PIDController controller;
    public static double p = 0.015, i = 0.004, d = 0.001;
    public static double f = 0;

    private final double ticks_in_degrees = 384.5/360;

    private final DcMotor turret;

    public Turret(HardwareMap hardwareMap) {
      turret = hardwareMap.dcMotor.get("turret");
      controller = new PIDController(p, i ,d);
    }

    public void toMotorPosition(int targetMotorPosition) {
        controller.setPID(p, i, d);
        int motorPosition = turret.getCurrentPosition();

        double pid = controller.calculate(motorPosition, targetMotorPosition);
        double ff = 0; // = Math.cos(Math.toRadians(targetMotorPosition/ticks_in_degrees)) * f;

        double power = pid + ff;
        turret.setPower(power);
    }

    public void resetTurretEncoder(){
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public double getTurretPosition() {
        return turret.getCurrentPosition();
    }

    public void faze_jarvis() {
        // assumes (0,0) is in the the top right corner
        double robot_x = -3;
        double robot_y = 0.1;

        double angle = Math.atan(robot_x/robot_y);
    }
}

