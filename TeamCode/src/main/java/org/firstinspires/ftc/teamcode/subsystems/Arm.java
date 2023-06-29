package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotor;

public class Arm {
    private DcMotor liftMotor;
    private CRServoImplEx slideMotor;
    public Arm(DcMotor liftMotor,CRServoImplEx slideMotor)
    {
        this.liftMotor = liftMotor;
        this.slideMotor = slideMotor;
    }

    public void lowSlide()
    {
        slideMotor.setPower(1);
    }

    public void medSlide()
    {
        slideMotor.setPower(1);
    }

    public void highSlide()
    {
        slideMotor.setPower(1);
    }

    public void liftUp()
    {
        liftMotor.setPower(1);
    }

    public void liftDown();
    {
        liftMotor.setPower(-1);
    }
}
