package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Slides {
    private Servo slides;
    public Slides(HardwareMap hardwareMap){
        slides = hardwareMap.servo.get("slides");
    }
    public void slidesForward(){

        slides.setPosition(.7);
    }
    public void slidesBackward(){
        slides.setPosition(.95);
    }
    public double getSlidesPosition(){
        return slides.getPosition();
    }
}
