package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
//import org.firstinspires.ftc.teamcode.commands.VoltageReader;
//import org.checkerframework.checker.units.qual.C;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.ControllerFeatures;
import org.firstinspires.ftc.teamcode.subsystems.Turrent;
import org.firstinspires.ftc.teamcode.subsystems.Wheels;


/*
    The reason why this class has OpMode instead of LinearOpMode is because
    equivalent would be to update the IMU in the "opModeIsActive" method
 */
@TeleOp(name="FieldCentricTeleOp", group="DriveModes")
public class FieldCentricTeleOp extends OpMode {
    double mult = 0.70;
    private Wheels wheels;
    //private Arm arm = new Arm();
    private Claw claw;
    private Turrent turret;
    private ControllerFeatures features;
    private GamepadEx pilot, sentry;
    private ElapsedTime runTime;

    @Override
    public void init() {
        runTime = new ElapsedTime();

        pilot = new GamepadEx(gamepad1);
        sentry = new GamepadEx(gamepad2);
        wheels = new Wheels(hardwareMap);

        telemetry.addLine("Initializatizing Robot");
        telemetry.update();
    }


    public void start() {
        runTime.reset();
    }

    @Override
    public void loop() {
        // color
        //features.setPurple(gamepad1, gamepad2, 100000);
        //features.setPurple(gamepad1, gamepad2, 100000);

        // this makes GamepadEx work
        pilot.readButtons();
        sentry.readButtons();
    }

    @Override
    public void stop() {
        telemetry.addLine("Robot Shut Down.");
        telemetry.addLine("Total Runtime: " + runTime + " seconds.");
        telemetry.addLine("Total Runtime: " + runTime + " seconds.");
        telemetry.update();
    }
}
