package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.ControllerFeatures;

/*
    The reason why this class has OpMode instead of LinearOpMode is because
    there are different methods in the OP mode, such as the loop() method.
    Since this method pulls from the Wheels subsystem, it requires a loop
    to constantly update the IMU inside the subsystem. In LinearOpMode, the
    equivalent would be to update the IMU in the "opModeIsActive" method
 */


@TeleOp(name="FieldCentricDrive", group="DriveModes")
public abstract class FieldCentricDrive extends LinearOpMode {

    // sets controller colors- find in Subsystem ControllerLights
    ControllerFeatures lights = new ControllerFeatures();

    @Override
    public void runOpMode() throws InterruptedException {



        waitForStart();

        if (isStopRequested())
            return;

        while (opModeIsActive()) {


        }

    }
}