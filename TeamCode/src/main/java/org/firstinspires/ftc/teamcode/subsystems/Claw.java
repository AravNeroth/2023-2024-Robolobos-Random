package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.ServoImplEx;

public class Claw {

    /*
     for Axon, find info how to use here
     https://docs.axon-robotics.com/servo/axon-servo-programmer

     and some more if you think you are him
     https://docs.axon-robotics.com/servo/advanced-tips

     to instantiate an Axon Servo in CR or Servo Mode:
     ServoImplEx normalServoMode = hardwareMap.get(ServoImplEx.class, "normal");
     CRServoImplEx crServoMode = hardwareMap.get(CRServoImplEx.class, "crMode");

     if Servo is in Continuous Rotational (CR) mode, it uses setPower(), else use setPosition()
     test the servo to test where the servo goes when value is set from -1 <--> 1
    */

    ServoImplEx leftClaw = hardwareMap.get(ServoImplEx.class, "leftClaw");
    ServoImplEx rightClaw = hardwareMap.get(ServoImplEx.class, "rightClaw");



    public void open(){
        rightClaw.setPosition(1);
        leftClaw.setPosition(1);
    }

    public void close(){

    }
}
