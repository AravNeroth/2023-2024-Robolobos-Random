package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
//import org.firstinspires.ftc.teamcode.commands.VoltageReader;
import org.checkerframework.checker.units.qual.C;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
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
@TeleOp(name="FieldCentricDrive", group="DriveModes")
public abstract class FieldCentricTeleOp extends OpMode {

    double mult = 0.70;
    // sets controller colors- find in Subsystem ControllerLights
    Wheels wheels;
    Arm arm;
    Claw claw;
    Turrent turret;
    ControllerFeatures features;
    
    GamepadEx pilot, sentry;
    private ElapsedTime runTime;
    @Override
    public void init(){

        pilot = new GamepadEx(gamepad1);
        sentry = new GamepadEx(gamepad2);

        wheels = new Wheels();
        arm = new Arm();
        turret = new Turrent();
        features = new ControllerFeatures();
        runTime = new ElapsedTime();

        /*
            when entering in gamepad1 & gamepad 2, it should be used for LED and Rumble only!
            For getting DATA use the GamepadEx class from FTClib
            https://docs.ftclib.org/ftclib/features/gamepad-extensions

            for some reason GamepadEx doesn't play nice with Gamepad, even though its a wrapper class
         */

        features.rumbleOnStart(gamepad1, gamepad2);
        features.setPink(gamepad1, gamepad2, 120);

        telemetry.addLine("ALl Subsystems & Controllers Actvated");
        telemetry.addLine("Initialization Completed Successfully.");
        telemetry.addLine("Time taken: " + getRuntime()+ " seconds.");
        telemetry.update();
    }

    @Override
    public void loop() {
        pilot.readButtons();
        sentry.readButtons();

        // multplier for the wheels- currently running @ 70%
        wheels.fieldCentric(pilot);

        // color
        features.setPurple(gamepad1, gamepad2, 100000);


        // speeding up controls
        // if the trigger is pressed halfway, then it'll boost
        if ((pilot.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.5) && (pilot.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) <= 0.6)) {
            wheels.setMult(0.75);
            features.lightRumble(gamepad1, 100);
        } else if ((pilot.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.6) && (pilot.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) <= 0.7)) {
            wheels.setMult(0.8);
            features.lightRumble(gamepad1, 100);
        } else if ((pilot.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.7) && (pilot.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) <= 0.8)) {
            wheels.setMult(0.85);
            features.lightRumble(gamepad1, 100);
        } else if ((pilot.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.8) && (pilot.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) <= 0.9)) {
            wheels.setMult(0.9);
            features.lightRumble(gamepad1, 100);
        } else if ((pilot.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.9) && (pilot.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) <= 1)) {
            wheels.setMult(1);
            features.lightRumble(gamepad1, 100);
        } else {
            wheels.setMult(0.7);
        }


        // slowing down controls
        // depending how far the trigger is held, the speed will decrease
        if ((pilot.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.5) && (pilot.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) <= 0.6)){
            wheels.setMult(0.65);
            features.lightRumble(gamepad1, 100);
        }

        else if((pilot.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.6) && (pilot.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) <= 0.7 )) {
            wheels.setMult(0.6);
            features.lightRumble(gamepad1, 100);
        }

        else if((pilot.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.7) && (pilot.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) <= 0.8 )) {
            wheels.setMult(0.55);
            features.lightRumble(gamepad1, 100);
        }

        else if((pilot.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.8) && (pilot.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) <= 0.9 )) {
            wheels.setMult(0.5);
            features.lightRumble(gamepad1, 100);
        }

        else if((pilot.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.9) && (pilot.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) <= 1 )) {
            wheels.setMult(0.45);
            features.lightRumble(gamepad1, 100);
        }

            // remember that the else statement is the default if the first if statement is false
        else
            wheels.setMult(0.7);



            // IMU reset
        if(pilot.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
            wheels.resetIMU();
            features.lightRumble(gamepad1, 100);
        }

        // turret controls
        if (sentry.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT))
            turret.turnRight();

        if (sentry.wasJustPressed(GamepadKeys.Button.DPAD_LEFT))
            turret.turnLeft();


        

    }

    @Override
    public void stop()
    {
        telemetry.addLine("Robot Stopped.");
        telemetry.addLine("Total Runtime: " + runTime + " seconds.");
        telemetry.update();
    }


}