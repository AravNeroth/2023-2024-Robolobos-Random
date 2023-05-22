package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.linearOpMode;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.opMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys.*;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;

import java.util.List;

/*
DISCLAIMER- I HAVE NO IDEA WHAT IM DOING
 */
public class Wheels{

    public RevIMU imu;

    double mult = 0.7;
    double avgVel;
    double flPower, frPower, blPower, brPower;

    List<LynxModule> hubs = hardwareMap.getAll(LynxModule.class);

    MotorEx FL = new MotorEx(hardwareMap, "leftFront");
    MotorEx BL = new MotorEx(hardwareMap,"leftRear");
    MotorEx FR = new MotorEx(hardwareMap,"rightFront");
    MotorEx BR = new MotorEx(hardwareMap,"rightRear");

    public Wheels() {

    FR.setInverted(true);
    BR.setInverted(true);

    //IMU
        imu = new RevIMU(hardwareMap);
        imu.init();
    // Without this, data retrieving from the IMU throws an exception

    }

    public void resetIMU(){
        imu.reset();
    }

    public void setMult(double mult){
        this.mult = mult;
    }

    public double getHeading(){
        return imu.getHeading();
    }

    public void passiveBrake(){
        /*
        checking if every single motor is 0 is the surest way to make
        sure the brake happens at the correct time.

        Since this method is being looped while the robot is on, it will
        always be checking if the power on every single wheel is 0

        (still theoretically, its untested)
        */
        if(frPower == 0 && flPower == 0 && brPower == 0 && blPower == 0) {
            FR.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
            FL.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
            BR.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
            BL.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        }
    }

    public void manualBrake(){
        FR.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        FL.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
    }

    // Dealing in hopes and prayers because yet again idk if this works
    public double getAvgVelocity(){
        avgVel = getAvgVelocity();
        return avgVel;
    }

    public void fieldCentric(GamepadEx gamepad){

        // i stole this what are these numbers at the end

        double y = Math.pow(gamepad.getLeftY(), 3);
        double x = Math.pow(gamepad.getLeftX() * 1.1, 3);
        double rx = Math.pow(gamepad.getRightX(), 3);

        double heading = Math.toRadians(-imu.getHeading()+180);
        double rotX = x * Math.cos(heading) - y * Math.sin(heading);
        double rotY = x * Math.sin(heading) + y * Math.cos(heading);

        flPower = (rotY + rotX + rx);
        blPower = (rotY - rotX + rx);
        frPower = (rotY - rotX - rx);
        brPower = (rotY + rotX - rx);


        // * ask john which one of the formulas i use for power cause iont know

        /*
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

        flPower = 1 * (rotY + rotX + rx) / denominator;
        blPower = 1 * (rotY - rotX + rx) / denominator;
        frPower = 1 * (rotY - rotX - rx) / denominator;
        brPower = 1 * (rotY + rotX - rx) / denominator;
        */

    }

    public void runMotors(){
        FL.setVelocity(flPower * mult);
        FR.setVelocity(frPower * mult);
        BL.setVelocity(blPower * mult);
        BR.setVelocity(brPower * mult);
    }


}
