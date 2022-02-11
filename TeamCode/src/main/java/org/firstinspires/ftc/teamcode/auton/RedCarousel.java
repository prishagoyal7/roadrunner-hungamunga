package org.firstinspires.ftc.teamcode.auton;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous(group = "ff")

public class RedCarousel extends LinearOpMode {
    private Servo carouselServo = null;
    @Override
    public void runOpMode() throws InterruptedException{
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        carouselServo = hardwareMap.get(Servo.class, "carousel1");

        waitForStart();

        if (isStopRequested()) return;
        Pose2d startingPose = new Pose2d(-36, -66, Math.toRadians(270));

        drive.setPoseEstimate(startingPose);
        Trajectory traj1 = drive.trajectoryBuilder(startingPose)
                .lineToLinearHeading(new Pose2d(-63,-66, Math.toRadians(0)))
                .build();

        Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                .splineTo(new Vector2d(-72,-36), Math.toRadians(0))
                .build();

        drive.followTrajectory(traj1);
        // run carousel
        carouselServo.setPosition(1);
        sleep(4000);
        drive.followTrajectory(traj2);

    }
}
