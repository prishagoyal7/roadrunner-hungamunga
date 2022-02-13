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
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(group = "ff")

public class RedCarousel extends LinearOpMode {
    private Servo carouselServo = null;
    @Override
    public void runOpMode() throws InterruptedException{
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        carouselServo = hardwareMap.get(Servo.class, "carousel2");

        waitForStart();

        if (isStopRequested()) return;
        Pose2d startingPose = new Pose2d(-36, 66, Math.toRadians(270));

        drive.setPoseEstimate(startingPose);
        TrajectorySequence seq1 = drive.trajectorySequenceBuilder(startingPose)
                .forward(5)
                .turn(Math.toRadians(-95))
                .back(20)
                .build();

        TrajectorySequence seq2 = drive.trajectorySequenceBuilder(seq1.end())
                .turn(Math.toRadians(-90))
                .back(18)
                .build();

//        Trajectory traj1 = drive.trajectoryBuilder(startingPose)
//                .lineTo(new Vector2d(-66,66))
//                .build();
//
//        Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
//                .strafeTo(new Vector2d(-66,40))
//                .build();


        drive.followTrajectorySequence(seq1);
        // run carousel
        carouselServo.setPosition(.7);
        sleep(5000);
        drive.followTrajectorySequence(seq2);


    }
}
