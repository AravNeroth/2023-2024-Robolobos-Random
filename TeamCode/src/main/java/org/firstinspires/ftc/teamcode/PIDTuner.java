package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Config
@TeleOp(name="PID Tuner", group="DriveModes")
public class PIDTuner extends OpMode {
    private PIDController controller;
    public static double p = 0, i = 0, d = 0;
    public static double f = 0;

    public static int target = 0;

    private final double ticks_in_degrees = 1425.1/360;

    private DcMotor motor;

    @Override
    public void init() {
        controller = new PIDController(p, i ,d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        motor = hardwareMap.get(DcMotor.class, "arm");
    }

    @Override
    public void loop() {
        controller.setPID(p, i, d);
        int motorPosition = motor.getCurrentPosition();

        double pid = controller.calculate(motorPosition, target);
        double ff = Math.cos(Math.toRadians(target/ticks_in_degrees)) * f;

        double power = pid + ff;

        motor.setPower(power);

        telemetry.addData("pos " , motorPosition);
        telemetry.addData("target ", target);
        telemetry.addData("power ", motor.getPower());
        telemetry.update();
    }
}
