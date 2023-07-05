package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.hardware.bosch.BNO055IMU;

public class Wheels{
    double multi = 0.7;
    double flPower, frPower, blPower, brPower;
    DcMotor FL, BL, FR, BR;

    BNO055IMU imu;
    BNO055IMU.Parameters parameters;

    public Wheels(HardwareMap hardwareMap) {
        FL = hardwareMap.dcMotor.get("leftFront");
        BL = hardwareMap.dcMotor.get("leftRear");
        FR = hardwareMap.dcMotor.get("rightFront");
        BR = hardwareMap.dcMotor.get("rightRear");

        FR.setDirection(DcMotorSimple.Direction.REVERSE);
        BR.setDirection(DcMotorSimple.Direction.REVERSE);

        // will set the behaviour of passive braking on start (theoretically)
        // the manual braking thing also didnt make much sense i didnt read the doc fully

        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //IMU
        //imu = new RevIMU(hardwareMap);
        //imu.init();
        imu = hardwareMap.get(BNO055IMU.class, "cIMU");
        parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);
        // Without this, data retrieving from the IMU throws an exception
    }

    public void resetIMU(){
        imu.initialize(parameters);
    }

    public void setMulti(double multi){
        this.multi = multi;
    }





    public void fieldCentric(GamepadEx gamePad){

        double y = -gamePad.getLeftY();
        double x = -gamePad.getLeftX();
        double rx = -gamePad.getRightX();

        double heading = -imu.getAngularOrientation().firstAngle;
        double rotX = x * Math.cos(heading) - y * Math.sin(heading);
        double rotY = x * Math.sin(heading) + y * Math.cos(heading);

        //flPower = (rotY + rotX + rx);
        //blPower = (rotY - rotX + rx);
        //frPower = (rotY - rotX - rx);
        //brPower = (rotY + rotX - rx);

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio, but only when
        // at least one is out of the range [-1, 1]
        // the 1 that ks being multiplied can be changed for drift correction
        // multi changes the speed of the motors in terms of %


        // * ask john which one of the formulas i use for power cause i don't know


        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

        flPower = 1 * (rotY + rotX + rx) / denominator;
        blPower = 1 * (rotY - rotX + rx) / denominator;
        frPower = 1 * (rotY - rotX - rx) / denominator;
        brPower = 1 * (rotY + rotX - rx) / denominator;


    }

    public void runMotors(){
        FL.setPower(flPower * multi);
        FR.setPower(frPower * multi);
        BL.setPower(blPower * multi);
        BR.setPower(brPower * multi);
    }
}
