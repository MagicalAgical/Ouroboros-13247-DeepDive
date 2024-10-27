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
public class ObservationSide extends LinearOpMode {
    private DcMotor leftLift = null;
    private DcMotor rightLift = null;
    private Servo claw = null;
    private DcMotor Arm = null;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addLine("Waiting for Start");
        telemetry.update();

        claw = hardwareMap.get(Servo.class, "claw");
        leftLift = hardwareMap.get(DcMotor.class, "leftLift");
        rightLift = hardwareMap.get(DcMotor.class, "rightLift");
        Arm = hardwareMap.get(DcMotor.class, "Arm");
        int x = 0;
        waitForStart();

        if (opModeIsActive()) {
            SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
            TrajectorySequence Left = drive.trajectorySequenceBuilder(new Pose2d())
                    .forward(15)
                    .lineToLinearHeading(new Pose2d(-94,1,Math.toRadians(135)))
                    .build();

            TrajectorySequence Left1 = drive.trajectorySequenceBuilder(new Pose2d())
                    .turn(Math.toRadians(45))
                    .strafeLeft(100)
                    .build();

            drive.followTrajectorySequence(Left);
            drive.followTrajectorySequence(Left1);
        }

    }
}