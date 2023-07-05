package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(group = "drive")
public class testingRR extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        waitForStart();

        if (isStopRequested()) return;

        drive.setPoseEstimate(new Pose2d(0,0, Math.toRadians(0)));

        Trajectory traj = drive.trajectoryBuilder(new Pose2d())
                .splineTo(new Vector2d(40, 30), 0)
                .build();

        Trajectory traj1 = drive.trajectoryBuilder(traj.end(), true)
                .splineTo(new Vector2d(0, 0), Math.toRadians(180))
                .build();

        Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                .splineTo(new Vector2d(40, 30), 0)
                .build();

        Trajectory traj3 = drive.trajectoryBuilder(traj2.end(), true)
                .splineTo(new Vector2d(0, 0), Math.toRadians(180))
                .build();

        Trajectory traj4 = drive.trajectoryBuilder(traj3.end())
                .splineTo(new Vector2d(40, 30), 0)
                .build();

        Trajectory traj5 = drive.trajectoryBuilder(traj4.end(), true)
                .splineTo(new Vector2d(0, 0), Math.toRadians(180))
                .build();

        Trajectory traj6 = drive.trajectoryBuilder(traj5.end())
                .splineTo(new Vector2d(40, 30), 0)
                .build();

        Trajectory traj7 = drive.trajectoryBuilder(traj6.end(), true)
                .splineTo(new Vector2d(0, 0), Math.toRadians(180))
                .build();

        Trajectory traj8 = drive.trajectoryBuilder(traj7.end())
                .splineTo(new Vector2d(40, 30), 0)
                .build();

        Trajectory traj9 = drive.trajectoryBuilder(traj8.end(), true)
                .splineTo(new Vector2d(0, 0), Math.toRadians(180))
                .build();

        drive.followTrajectory(traj);
        drive.followTrajectory(traj1);
        drive.followTrajectory(traj2);
        drive.followTrajectory(traj3);
        drive.followTrajectory(traj4);
        drive.followTrajectory(traj5);
        drive.followTrajectory(traj6);
        drive.followTrajectory(traj7);
        drive.followTrajectory(traj8);
        drive.followTrajectory(traj9);

        sleep(1500);
    }
}
