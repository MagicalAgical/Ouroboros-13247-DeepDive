package org.firstinspires.ftc.teamcode;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;
import org.openftc.easyopencv.OpenCvPipeline;
@Autonomous
public class RedLeft extends LinearOpMode{


    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addLine("Waiting for Start");
        telemetry.update();
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(0,0,0);
        drive.setPoseEstimate(startPose);


        Trajectory myTrajectory = drive.trajectoryBuilder(new Pose2d())
                .forward(15)
                .back(5)
                .build();


        Trajectory myTraj2 = drive.trajectoryBuilder(new Pose2d())
                .lineToLinearHeading(new Pose2d(15,0,Math.toRadians(90)))
                .build();



        waitForStart();

        if(isStopRequested()) return;

        drive.followTrajectory(myTrajectory);
        drive.followTrajectory(myTraj2);

    }
}