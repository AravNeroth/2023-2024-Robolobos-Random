package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Slides {
    private CRServo slides;
    public Slides(HardwareMap hardwareMap){
        slides = hardwareMap.crservo.get("slides");
    }
    public void slidesForward(){
        slides.setPower(.8);
    }
}
