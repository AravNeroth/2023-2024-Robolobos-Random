package org.firstinspires.ftc.teamcode.subsystems;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad2;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.hardware.Gamepad;

public class ControllerFeatures {

    int ms;

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

    public void rumble(int seconds){
        ms = (seconds * 1000);
    }
}
