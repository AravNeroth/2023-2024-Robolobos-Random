package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.hardware.lynx.LynxVoltageSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.CRServoImpl;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import com.acmerobotics.dashboard.*;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.*;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.teamcode.subsystems.ControllerFeatures;

/*
Created by Arav Neroth on 9/8/2023, for FTC Teams 14361 & 14363
Intention: Voltage testbed, hopes of using voltage to manipulate
Drive Constant variables during Auton so that it is more consistant


(i lied abt the type of voltage sensor lmao, i changed my mind after i made the file)
 */

@TeleOp(name="VoltageTester", group="DriveModes")
public class LynxVoltageTester extends LinearOpMode{

    // this changes the speed multiplier for wheels
    double mult = 0.5;
    long timeTaken;

    ElapsedTime clock;
    LynxVoltageSensor voltageReader;
    double frontLeftPower, frontRightPower, rearLeftPower, rearRightPower;

    //Gamepad gamepad1;

    DcMotorEx motor1, motor2, motor3, motor4;

    @Override
    public void runOpMode() throws InterruptedException {

        DcMotorEx motor1 = (DcMotorEx) hardwareMap.dcMotor.get("motor1");
        motor1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        DcMotorEx motor2 = (DcMotorEx) hardwareMap.dcMotor.get("motor2");
        motor2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        DcMotorEx motor3 = (DcMotorEx) hardwareMap.dcMotor.get("motor3");
        motor3.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        DcMotorEx motor4 = (DcMotorEx) hardwareMap.dcMotor.get("motor4");
        motor4.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);


        motor2.setDirection(DcMotorSimple.Direction.REVERSE);
        motor4.setDirection(DcMotorSimple.Direction.REVERSE);


        //Servo axonTest = hardwareMap.servo.get("axon1");



// ------------- Below is the telementry for voltage ---------------- \\

        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        telemetry.addLine("Rev Motor Initialized.");
        telemetry.addLine("Qualcomm Voltage Sensor Intializing.");
        telemetry.update();


        telemetry.addLine("Qualcomm Voltage Sensor Intialized.");
        telemetry.addData("Voltage on Initialization: ", "%.1f volts.", new Func<Double>() {
            public Double value() {
                return getBatteryVoltage();
            }
        });
        telemetry.update();

// ----------- Above is the telementry for voltage ---------------- \\


        // imports the IMU
        BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        // this is the default & unneeded, however specifying is easier to see the steps
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        // Without this, data retrieving from the IMU throws an exception
        imu.initialize(parameters);

        ControllerFeatures feature = new ControllerFeatures();

        ElapsedTime clock = new ElapsedTime();

        // rumbles on initialization
        feature.rumbleOnStart(gamepad1, gamepad2);
        feature.setPink(gamepad1, gamepad2, 10);

        clock.reset();


        waitForStart();

        if (isStopRequested()) return;


        while(opModeIsActive()) {

            double leftX = gamepad1.left_stick_x;
            double leftY = -gamepad1.left_stick_y; 
            double rightX = gamepad1.right_stick_x;

            // Calculate motor powers based on joystick input
            // m1, m2, m3, m4
            frontLeftPower = leftY - leftX - rightX;
            frontRightPower = leftY + leftX + rightX;
            rearLeftPower = leftY + leftX - rightX;
            rearRightPower = leftY - leftX + rightX;


            // ------------- Below is the telementry for voltage ---------------- \\

            telemetry.addData("Current Voltage Levels: ", "%.1f volts.", new Func<Double>() {
                public Double value() {
                    return getBatteryVoltage();
                }
            });
            telemetry.update();

            // ----------- Above is the telementry for voltage ---------------- \\

            if(getBatteryVoltage() > 13.5){
                mult = 1;
            } else if (getBatteryVoltage() < 13.5) {
                mult = 0.25;
            }

            // imu reset is dpad up
            if (gamepad1.dpad_up) {
                imu.initialize(parameters);
            }


            telemetry.addData("Motor 1 (FL): ", motor1.getVelocity());
            telemetry.addData("Motor 2 (FR): ", motor2.getVelocity());
            telemetry.addData("Motor 3 (BL): ", motor3.getVelocity());
            telemetry.addData("Motor 4 (BR): ", motor4.getVelocity());

            telemetry.update();

            // this will run the test and return how long it takes
            if(gamepad1.x) {
                telemetry.addData("Time Taken to Run To 500 Ticks: ", timeTest());
                telemetry.update();
                motor1.setMotorDisable();
                motor2.setMotorDisable();
                motor3.setMotorDisable();
                motor4.setMotorDisable();
            }

        }

    }

    public double getBatteryVoltage() {
        double result = Double.POSITIVE_INFINITY;
        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            double voltage = sensor.getVoltage();
            if (voltage > 0) {
                result = Math.min(result, voltage);
            }
        }
        return result;
    }

    public double timeTest(){

        motor1.setTargetPosition(500);
        motor2.setTargetPosition(500);
        motor3.setTargetPosition(500);
        motor4.setTargetPosition(500);
        clock.reset();
        motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor3.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor4.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        powerMotors();

        if (!motor1.isBusy())
            return clock.time();

        return 0.0;
    }

    public void powerMotors(){
        motor1.setPower(frontLeftPower);
        motor2.setPower(frontRightPower);
        motor3.setPower(rearLeftPower);
        motor4.setPower(rearRightPower);

    }


}
