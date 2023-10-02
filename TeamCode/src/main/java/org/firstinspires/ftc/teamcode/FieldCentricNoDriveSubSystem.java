package org.firstinspires.ftc.teamcode;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;

import com.qualcomm.hardware.lynx.LynxVoltageSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;
import org.firstinspires.ftc.teamcode.subsystems.ControllerFeatures;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;

@TeleOp(name="FieldCentricNoDriveSubSystem", group="DriveModes")
public class FieldCentricNoDriveSubSystem extends LinearOpMode{

    // this changes the speed multiplier for wheels
    double mult = 0.75;
    private GamepadEx pilot, sentry;
    double encoderValue;

    VoltageSensor voltSensor;


    @Override
    public void runOpMode() throws InterruptedException {

        DcMotor FL = hardwareMap.dcMotor.get("fl");
        DcMotor BL = hardwareMap.dcMotor.get("bl");
        DcMotor FR = hardwareMap.dcMotor.get("fr");
        DcMotor BR = hardwareMap.dcMotor.get("bl");

        FR.setDirection(DcMotorSimple.Direction.REVERSE);
        BR.setDirection(DcMotorSimple.Direction.REVERSE);

        FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        voltSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");
        pilot = new GamepadEx(gamepad1);
        sentry = new GamepadEx(gamepad2);
        

        ControllerFeatures feature = new ControllerFeatures();

        // rumbles on initialization
        feature.rumbleOnStart(gamepad1, gamepad2);
        feature.setPink(gamepad1, gamepad2, 10);

        waitForStart();

        if (isStopRequested()) return;

        while(opModeIsActive()) {

            telemetry.addData("Encoder Position: ", FR.getCurrentPosition());
            telemetry.addData("Current Voltage: ", voltSensor.getVoltage());
            telemetry.update();


            if(gamepad1.left_trigger > 0.8){
                feature.lightRumble(gamepad1, gamepad2, 500);
            }

            else if(gamepad1.right_trigger > 0.8){
                feature.lightRumble(gamepad1, gamepad2, 500);
            }



        }

    }

}
