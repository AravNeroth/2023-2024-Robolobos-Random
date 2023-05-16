package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
//import org.firstinspires.ftc.teamcode.commands.VoltageReader;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.ControllerFeatures;
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
    Wheels wheels = new Wheels();
    Arm arm = new Arm();
    Claw claw = new Claw();
    ControllerFeatures features = new ControllerFeatures();

    //private VoltageReader voltage;
    private ElapsedTime runTime;
    @Override
    public void init(){
        runTime = new ElapsedTime();


        features.rumbleOnStart(gamepad1, gamepad2);
        features.setRainbow(gamepad1, gamepad2);

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
            features.lightRumble(gamepad1, gamepad2, 100);
        }

        else if(gamepad1.right_trigger > 1){
            mult = 0.5;
        }

        else{
            mult = 0.70;
        }

        if(gamepad1.dpad_up){
            wheels.resetIMU();
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