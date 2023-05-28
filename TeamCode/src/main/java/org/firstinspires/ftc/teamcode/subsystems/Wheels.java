package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.linearOpMode;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.opMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys.*;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;

/*
DISCLAIMER- I HAVE NO IDEA WHAT IM DOING
 */
public class Wheels{

    private RevIMU imu;

    double mult = 0.7;
    double flPower, frPower, blPower, brPower;
    DcMotor FL = hardwareMap.dcMotor.get("leftFront");
    DcMotor BL = hardwareMap.dcMotor.get("leftRear");
    DcMotor FR = hardwareMap.dcMotor.get("rightFront");
    DcMotor BR = hardwareMap.dcMotor.get("rightRear");

    public Wheels() {

    FR.setDirection(DcMotorSimple.Direction.REVERSE);
    BR.setDirection(DcMotorSimple.Direction.REVERSE);

    //IMU
        imu = new RevIMU(hardwareMap);
        imu.init();
    // Without this, data retrieving from the IMU throws an exception

        // will set the behaviour of passive braking on start (theoretically)
        // the manual braking thing also didnt make much sense i didnt read the doc fully

        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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



    public void fieldCentric(GamepadEx gamepad){

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

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio, but only when
        // at least one is out of the range [-1, 1]
        // the 1 that ks being multiplied can be changed for drift correction
        // mult changes the speed of the motors in terms of %


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
        FL.setPower(flPower * mult);
        FR.setPower(frPower * mult);
        BL.setPower(blPower * mult);
        BR.setPower(brPower * mult);
    }


}
