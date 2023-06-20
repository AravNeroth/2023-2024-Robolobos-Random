package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.subsystems.Wheels;
import org.firstinspires.ftc.teamcode.subsystems.Slides;

/*
    The reason why this class has OpMode instead of LinearOpMode is because
    there are different methods in the OP mode, such as the loop() method.
    Since this method pulls from the Wheels subsystem, it requires a loop
    to constantly update the IMU inside the subsystem. In LinearOpMode, the
    equivalent would be to update the IMU in the "opModeIsActive" method
 */
@TeleOp(name="FieldCentricTeleOp", group="DriveModes")
public class FieldCentricTeleOp extends OpMode {
    private Wheels wheels;
    private Turret turret;
    private Arm arm;
    private GamepadEx pilot, sentry;
    private ElapsedTime runTime;
    //private Slides slides;
    private double turnPower;
    private final int limit = 1200;

    @Override
    public void init() {
        runTime = new ElapsedTime();

        pilot = new GamepadEx(gamepad1);
        sentry = new GamepadEx(gamepad2);
        wheels = new Wheels(hardwareMap);
        turret = new Turret(hardwareMap);
        arm = new Arm(hardwareMap);
        //slides = new Slides(hardwareMap);

        telemetry.addLine("Initializing Robot");
        telemetry.update();
    }


    public void start() {
        runTime.reset();
        telemetry.addLine("Running");
        telemetry.update();
    }

    @Override
    public void loop() {
        telemetry.addLine("Running loop");
        telemetry.addData("Arm Motor Target Position: ", arm.getArmMotorTargetPosition());
        telemetry.addData("Arm Motor Power", arm.getArmMotorPower());
        telemetry.addData("Turret Motor Rotation", turnPower);
        telemetry.addData("Turret Location", turret.getTurretPosition());
        //telemetry.addData("Slides Location", slides.getSlidesPosition());
        telemetry.update();

        // this makes GamePadEx work
        pilot.readButtons();
        sentry.readButtons();

        wheels.fieldCentric(pilot);
        wheels.runMotors();

        if (sentry.gamepad.right_stick_x > 0 && turret.getTurretPosition() <= limit) {
            turret.turnWithTrigger(sentry.gamepad.right_stick_x);
        } else if (sentry.gamepad.right_stick_x < 0 && turret.getTurretPosition() >= -limit) {
            turret.turnWithTrigger(sentry.gamepad.right_stick_x);
        } else if (sentry.gamepad.dpad_down) {
            turret.presetTurretSide();
        } else if(sentry.gamepad.ps){
            turret.resetTurretEncoder();
        } else {
            turret.stopTurret();
        }

        if (sentry.gamepad.dpad_right) {
            arm.armUp();
        } else if (sentry.gamepad.dpad_left) {
            arm.armDown();
        } else if (sentry.gamepad.dpad_up) {
            arm.armMid();
        }
/*
        if (sentry.gamepad.right_bumper) {
            slides.slidesForward();
        } else if (sentry.gamepad.left_bumper) {
            slides.slidesBackward();
        }
       */
    }

    @Override
    public void stop() {
        telemetry.addLine("Robot Shut Down.");
        telemetry.addLine("Total Runtime: " + runTime + " seconds.");
        telemetry.addLine("Total Runtime: " + runTime + " seconds.");
        telemetry.update();
    }
}
