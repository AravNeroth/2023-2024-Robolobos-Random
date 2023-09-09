package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.lynx.LynxVoltageSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import com.acmerobotics.dashboard.*;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.*;
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

    DcMotorEx motor4;
    @Override
    public void runOpMode() throws InterruptedException {

        DcMotorEx motor4 = (DcMotorEx) hardwareMap.dcMotor.get("motor4");
        motor4.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);


// ------------- Below is the telementry for voltage ---------------- \\

        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        telemetry.addLine("Rev Motor Intialized.");
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
        // Technically this is the default, however specifying it is clearer
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


            telemetry.addData("Motor4 Velocity: ", motor4.getVelocity());
            telemetry.update();

            if(gamepad1.x) {
                telemetry.addData("Time Taken to Complete: ", timeTest());
                telemetry.update();
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

    public long timeTest(){

        motor4.setTargetPosition(500);
        clock.reset();
        motor4.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor4.setVelocity(mult);

        return timeTaken;
    }

}
