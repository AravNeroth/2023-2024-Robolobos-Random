package org.firstinspires.ftc.teamcode.subsystems;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad2;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.hardware.Gamepad;

/*
IMPORTANT**

THIS CLASS WILL PRETTY MUCH ONLY WORK WITH THE FIELDCENTRICNODIVESUBSYSTEM CODE BECAUSE IT
IS FOR LINEAROP MODE CLASSES! (will makae one for Op Mode)

why can't it work? gamepad1 and gamepad2 update at the same time, so it could change the loop
that OpMode has. LinearOp mode doesn't care about that since it runs once.
 */
public class ControllerFeatures {

    int ms;

    /*
     HOW TO USE RUMBLE EXAMPLE, ALMOST EXACT TO CONTROLLER LIGHTS
    (god i love these controllers)

    Gamepad.RumbleEffect effect = new Gamepad.RumbleEffect.Builder()
       .addStep(0.0, 1.0, 500)  //  Rumble right motor 100% for 500 mSec
       .addStep(0.0, 0.0, 300)  //  Pause for 300 mSec
       .addStep(1.0, 0.0, 250)  //  Rumble left motor 100% for 250 mSec
       .addStep(0.0, 0.0, 250)  //  Pause for 250 mSec
       .addStep(1.0, 0.0, 250)  //  Rumble left motor 100% for 250 mSec
       .build();

        gamepad1.runRumbleEffect(effect);
        gamepad2.runRumbleEffect(effect);

    */

    // rainbow wave (in theory)
    Gamepad.LedEffect rainbowManual = new Gamepad.LedEffect.Builder()
            .addStep(1, 0, 0, 7000) // Show red for 7s
            .addStep(255, 128, 0, 7000) // Show orange for 7s
            .addStep(255, 255, 51, 7000) // Show yellow for 7s
            .addStep(0, 1, 0, 7000) // Show green for 7s
            .addStep(0, 0, 1, 7000) // Show blue for 7s
            .addStep(102, 0, 204, 7000) // Show purple for 7s
            .addStep(1, 1, 1, 7000) // Show white for 7s
            .addStep(255, 51, 255, 50000) // Show pink for 50s

            .build();

    // startup build. rumbles right, then left, then both
    Gamepad.RumbleEffect startUp = new Gamepad.RumbleEffect.Builder()
            .addStep(0.0, 0.5, 800)  //  Rumble right motor 50% for 800 mSec
            .addStep(0.0, 0.0, 500)  //  Pause for 500 mSec
            .addStep(0.5, 0.0, 800)  //  Rumble left motor 50% for 800 mSec
            .addStep(0.0, 0.0, 500)  //  Pause for 500 mSec
            .addStep(1.0, 1.0, 1250)  //  Rumble both motors 100% for 1.25 Sec
            .build();

    public void setRainbow(){
        gamepad1.runLedEffect(rainbowManual);
        gamepad2.runLedEffect(rainbowManual);
    }

    public void setPurple(int seconds){
        ms = (seconds * 1000);
        gamepad1.setLedColor(102, 0, 204, ms);
        gamepad2.setLedColor(102, 0, 204, ms);
    }

    public void setBlue(int seconds){
        ms = (seconds * 1000);
        gamepad1.setLedColor(0, 0, 55, ms);
        gamepad2.setLedColor(0, 0, 55, ms);
    }

    public void setGreen(int seconds){
        ms = (seconds * 1000);
        gamepad1.setLedColor(0, 100, 0, ms);
        gamepad2.setLedColor(0, 100, 0, ms);
    }

    public void setPink(int seconds){
        ms = (seconds * 1000);
        gamepad1.setLedColor(255, 51, 255, ms);
        gamepad2.setLedColor(255, 51, 255, ms);

    }


    public void rumbleOnStart(){
        gamepad1.runRumbleEffect(startUp);
        gamepad2.runRumbleEffect(startUp);
    }
}
