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
    private DcMotorEx carouselMotor = null;
    @Override
    public void runOpMode() throws InterruptedException{
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        carouselMotor = hardwareMap.get(DcMotorEx.class, "carousel");
        carouselMotor.setDirection(DcMotorEx.Direction.FORWARD);

        waitForStart();

        if (isStopRequested()) return;
        Pose2d startingPose = new Pose2d(-36, -66, Math.toRadians(90));

        drive.setPoseEstimate(startingPose);
        Trajectory traj1 = drive.trajectoryBuilder(startingPose)
                .splineToSplineHeading(new Pose2d(-60,-56, Math.toRadians(0)), Math.toRadians(90))
                .build();

        Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                .lineTo(new Vector2d(-66,-56))
                .build();

        Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
                .strafeTo(new Vector2d(-66,-30))
                .build();


        drive.followTrajectory(traj1);
        // run carousel
        drive.followTrajectory(traj2);
        carouselMotor.setVelocity(2800 * -1);
        sleep(4000);
        drive.followTrajectory(traj3);

    }
}
