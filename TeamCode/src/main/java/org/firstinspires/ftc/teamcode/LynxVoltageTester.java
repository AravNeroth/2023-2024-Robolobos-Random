package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.lynx.LynxVoltageSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import com.acmerobotics.dashboard.*;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.*;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.ControllerFeatures;
import org.firstinspires.ftc.teamcode.subsystems.Turrent;

/*
Created by Arav Neroth on 9/8/2023, for FTC Teams 14361 & 14363
Intention: Voltage testbed, hopes of using voltage to manipulate
Drive Constant variables during Auton so that it is more consistant


(i lied abt the type of voltage sensor lmao, i changed my mind after i made the file)
 */

@TeleOp(name="LynxVoltageTester", group="DriveModes")
public class LynxVoltageTester extends LinearOpMode{

    // this changes the speed multiplier for wheels
    double mult = 0.75;
    LynxVoltageSensor voltageReader;

    @Override
    public void runOpMode() throws InterruptedException {

        DcMotor motor4 = hardwareMap.dcMotor.get("motor4");

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

        // rumbles on initialization
        feature.rumbleOnStart(gamepad1, gamepad2);
        feature.setPink(gamepad1, gamepad2, 10);

        waitForStart();

        if (isStopRequested()) return;

        while(opModeIsActive()) {

            telemetry.addData("Current Voltage Levels: ", "%.1f volts.", new Func<Double>() {
                public Double value() {
                    return getBatteryVoltage();
                }
            });
            telemetry.update();

            // sets controller colors- find in Subsystem ControllerLights
            feature.setRainbow(gamepad1, gamepad2);

            // imu reset is dpad up
            if (gamepad1.dpad_up) {
                imu.initialize(parameters);
            }

            if(gamepad1.left_trigger > 0.8){
                mult = 1;
                feature.lightRumble(gamepad1, gamepad2, 500);
            }

            else if(gamepad1.right_trigger > 0.8){
                mult = 0.5;
                feature.lightRumble(gamepad1, gamepad2, 500);
            }

            else{
                mult = 0.75;
            }


            motor4.setPower(mult * 1);

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

}
