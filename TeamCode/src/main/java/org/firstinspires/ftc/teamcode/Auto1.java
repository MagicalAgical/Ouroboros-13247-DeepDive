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
public class Auto1 extends LinearOpMode{
    private static DcMotor intakeOne = null;
    private static DcMotor intakeTwo = null;
    private DcMotor leftLift = null;
    private DcMotor rightLift = null;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addLine("Waiting for Start");
        telemetry.update();

        intakeOne = hardwareMap.get(DcMotor.class, "intakeOne");
        intakeTwo = hardwareMap.get(DcMotor.class, "intakeTwo");
        leftLift = hardwareMap.get(DcMotor.class, "leftLift");
        rightLift = hardwareMap.get(DcMotor.class, "rightLift");

        waitForStart();

        if (opModeIsActive()){
            SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
            TrajectorySequence Left = drive.trajectorySequenceBuilder(new Pose2d())
                    .forward(23)
                    .turn(Math.toRadians(135))
                    .forward(19)
                    .addTemporalMarker(5,()->{
                        intakeOne.setPower(0.8);
                        intakeTwo.setPower(-0.8);
                    })
                    .build();

            drive.followTrajectorySequence(Left);
        }
    }
}
