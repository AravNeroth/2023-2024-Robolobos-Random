package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.subsystems.IMUjeff;

public class Wheels{
    double multi = 0.7;
    double flPower, frPower, blPower, brPower;
    DcMotor FL, BL, FR, BR;

    IMUjeff chassisIMU;

    public Wheels(HardwareMap hardwareMap) {
        FL = hardwareMap.dcMotor.get("leftFront");
        BL = hardwareMap.dcMotor.get("leftRear");
        FR = hardwareMap.dcMotor.get("rightFront");
        BR = hardwareMap.dcMotor.get("rightRear");

        FR.setDirection(DcMotorSimple.Direction.REVERSE);
        BR.setDirection(DcMotorSimple.Direction.REVERSE);

        //IMU
        //imu = new RevIMU(hardwareMap);
        //imu.init();
        chassisIMU = new IMUjeff(hardwareMap, "cIMU");
        // Without this, data retrieving from the IMU throws an exception
    }

    public void resetIMU(){
        //imu.reset();
    }

    public void setMulti(double multi){
        this.multi = multi;
    }



    public void passiveBrake(){
        /*
        checking if every single motor is 0 is the surest way to make
        sure the brake happens at the correct time.

        Since this method is being looped while the robot is on, it will
        always be checking if the power on every single wheel is 0

        (still theoretically, its untested)
        */
        if(frPower == 0 && flPower == 0 && brPower ==0 && blPower == 0) {
            FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }

    public void manualBrake(){
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void fieldCentric(GamepadEx gamePad){

        double y = -gamePad.getLeftY();
        double x = -gamePad.getLeftX();
        double rx = -gamePad.getRightX();

        double heading = Math.toRadians(-chassisIMU.getYaw());
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
        // multi changes the speed of the motors in terms of %


        // * ask john which one of the formulas i use for power cause i don't know

        /*
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

        flPower = 1 * (rotY + rotX + rx) / denominator;
        blPower = 1 * (rotY - rotX + rx) / denominator;
        frPower = 1 * (rotY - rotX - rx) / denominator;
        brPower = 1 * (rotY + rotX - rx) / denominator;
        */

    }

    public void runMotors(){
        FL.setPower(flPower * multi);
        FR.setPower(frPower * multi);
        BL.setPower(blPower * multi);
        BR.setPower(brPower * multi);
    }
}
