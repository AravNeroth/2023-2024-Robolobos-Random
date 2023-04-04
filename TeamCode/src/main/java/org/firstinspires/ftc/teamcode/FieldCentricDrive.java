package org.firstinspires.ftc.teamcode;

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

    double mult = 0.70;
    // sets controller colors- find in Subsystem ControllerLights
    ControllerFeatures features = new ControllerFeatures();
    Wheels wheels = new Wheels();
    //private VoltageReader voltage;
    private ElapsedTime runTime;
    @Override
    public void init(){
        runTime = new ElapsedTime();


        features.rumbleOnStart();
        features.setRainbow();

        // this aint working fsr
    //    voltage = new VoltageReader(hardwareMap);

        telemetry.addLine("Initialization Completed Successfully.");
        telemetry.addLine("Time taken: " + getRuntime()+ " seconds.");
        telemetry.update();
    }

    @Override
    public void loop()
    {
        if(gamepad1.left_trigger > 1){
            mult = 1;
            features.lightRumble(500);
        }

        else if(gamepad1.right_trigger > 1){
            mult = 0.5;
        }

        else{
            mult = 0.70;
        }

        // multplier for the wheels- currently running @ 70%
        wheels.fieldCentric(mult);

    }

    @Override
    public void stop()
    {
        telemetry.addLine("Robot Stopped.");
        telemetry.addLine("Total Runtime: " + getRuntime() + " seconds.");
        telemetry.update();
    }


}