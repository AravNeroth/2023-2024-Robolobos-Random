package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.linearOpMode;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.opMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
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

    BNO055IMU imu;

    DcMotor FL = hardwareMap.dcMotor.get("front left");
    DcMotor BL = hardwareMap.dcMotor.get("back left");
    DcMotor FR = hardwareMap.dcMotor.get("front right");
    DcMotor BR = hardwareMap.dcMotor.get("back right");

    public Wheels() {

    FR.setDirection(DcMotorSimple.Direction.REVERSE);
    BR.setDirection(DcMotorSimple.Direction.REVERSE);

    //IMU
    // Retrieve the IMU from the hardware map
    imu = hardwareMap.get(BNO055IMU.class, "imu");
    // this is making a new object called 'parameters' that we use to hold the angle the imu is at
    BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
    // Technically this is the default, however specifying it is clearer
    parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
    // Without this, data retrieving from the IMU throws an exception
        imu.initialize(parameters);
    }


    public void fieldCentric(GamepadEx controller, double mult){

        double y = controller.getLeftY();
        double x = -controller.getLeftX();
        double rx = -controller.getRightX();

        double botHeading = -imu.getAngularOrientation().firstAngle;

        double rotX = x * Math.cos(botHeading) - y * Math.sin(botHeading);
        double rotY = x * Math.sin(botHeading) + y * Math.cos(botHeading);

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio, but only when
        // at least one is out of the range [-1, 1]
        // the 1 that ks being multiplied can be changed for drift correction
        // mult changes the speed of the motors in terms of %

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

        double frontLeftPower = 1 * (mult * (rotY + rotX + rx)) / denominator;
        double backLeftPower = 1 * (mult * (rotY - rotX + rx)) / denominator;
        double frontRightPower = 1 * (mult * (rotY - rotX - rx)) / denominator;
        double backRightPower = 1 * (mult * (rotY + rotX - rx)) / denominator;

        FL.setPower(frontLeftPower);
        BL.setPower(backLeftPower);
        FR.setPower(frontRightPower);
        BR.setPower(backRightPower);



    }
}
