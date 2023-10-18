package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
//import org.firstinspires.ftc.teamcode.commands.VoltageReader;
import org.firstinspires.ftc.teamcode.subsystems.ControllerFeatures;
import org.firstinspires.ftc.teamcode.subsystems.Wheels;
import com.qualcomm.robotcore.util.ElapsedTime;


/*
    The reason why this class has OpMode instead of LinearOpMode is because
    there are different methods in the OP mode, such as the loop() method.
    Since this method pulls from the Wheels subsystem, it requires a loop
    to constantly update the IMU inside the subsystem. In LinearOpMode, the
    equivalent would be to update the IMU in the "opModeIsActive" method
 */


@TeleOp(name="FieldCentricDrive", group="DriveModes")
public abstract class FieldCentricDrive extends OpMode {

    @Override
    public void init(){

    }

    @Override
    public void loop()
    {


    }

    @Override
    public void stop()
    {

    }


}