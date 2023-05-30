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
    there are different methods in the OP mode, such as the loop() method.
    Since this method pulls from the Wheels subsystem, it requires a loop
    to constantly update the IMU inside the subsystem. In LinearOpMode, the
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
        turret = new Turrent(hardwareMap);

        telemetry.addLine("Initializatizing Robot");
        telemetry.update();
    }


    public void start() {
        runTime.reset();
        telemetry.addLine("Running");
        telemetry.update();
    }

    @Override
    public void loop() {
        // color
        //features.setPurple(gamepad1, gamepad2, 100000);
        //features.setPurple(gamepad1, gamepad2, 100000);

        telemetry.addLine("Running loop");
        telemetry.update();

        // this makes GamepadEx work
        pilot.readButtons();
        sentry.readButtons();

        wheels.fieldCentric(pilot);
        wheels.runMotors();

        if (gamepad2.x) {
            turret.turnLeft();
        }

        if (gamepad2.b) {
            turret.turnRight();
        }
    }

    @Override
    public void stop() {
        telemetry.addLine("Robot Shut Down.");
        telemetry.addLine("Total Runtime: " + runTime + " seconds.");
        telemetry.addLine("Total Runtime: " + runTime + " seconds.");
        telemetry.update();
    }
}
