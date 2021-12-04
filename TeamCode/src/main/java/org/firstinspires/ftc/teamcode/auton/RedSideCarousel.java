//package org.firstinspires.ftc.teamcode.auton;
//
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.acmerobotics.roadrunner.geometry.Pose2d;
//import com.acmerobotics.roadrunner.geometry.Vector2d;
//import com.acmerobotics.roadrunner.trajectory.Trajectory;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//
//import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
//
//@Autonomous(group = "ff")
//
//public class RedSideCarousel extends LinearOpMode {
//    @Override
//    public void runOpMode() throws InterruptedException{
//        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
//
//        waitForStart();
//
//        if (isStopRequested()) return;
//        Pose2d startingPose = new Pose2d(-36, -66, Math.toRadians(90));
//
//        drive.setPoseEstimate(startingPose);
//        Trajectory traj1 = drive.trajectoryBuilder(startingPose)
//                .splineTo(new Vector2d(-12,-42), Math.toRadians(90))
//                .build();
//
//        Trajectory traj2 = drive.trajectoryBuilder(traj1.end(), true)
//                .splineToSplineHeading(new Pose2d(-66,-56, Math.toRadians(0)), Math.toRadians(90))
//                .build();
//
//        Trajectory traj3 = drive.trajectoryBuilder(traj2.end())// add a true
//                .splineTo(new Vector2d(-70,-36), Math.toRadians(0))
//                .build();
//
//        drive.followTrajectory(traj1);
//        // drop the block
//        drive.followTrajectory(traj2);
//        // run carousel
//        drive.followTrajectory(traj3);
//    }
//}
