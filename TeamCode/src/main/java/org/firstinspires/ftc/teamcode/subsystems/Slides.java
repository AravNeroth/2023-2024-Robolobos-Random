package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotor;

public class Slides
{
    private CRServoImplEx slide;
    private DcMotor liftMotor ;

    public Slides(CRServoImplEx slide, DcMotor liftMotor)
    {
        this.slide = slide;
        this.liftMotor = liftMotor;
    }

    public void lowSlide()
    {
            slide.setPower(1);
    }

    public void medSlide()
    {
        slide.setPower(1);
    }

    public void highSlide()
    {
        slide.setPower(1);
    }

    public void lift
}
