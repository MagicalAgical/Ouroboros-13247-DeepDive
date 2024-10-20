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
public class BasketSide extends LinearOpMode{
    private DcMotor leftLift = null;
    private DcMotor rightLift = null;
    private Servo claw = null;
    private DcMotor Arm = null;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addLine("Waiting for Start");
        telemetry.update();

        claw = hardwareMap.get(Servo.class,"claw");
        leftLift = hardwareMap.get(DcMotor.class, "leftLift");
        rightLift = hardwareMap.get(DcMotor.class, "rightLift");
        Arm = hardwareMap.get(DcMotor.class,"Arm");
        int x = 0;
        waitForStart();

        if (opModeIsActive()){
            SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
            TrajectorySequence Left = drive.trajectorySequenceBuilder(new Pose2d())
                    .forward(22)
                    .turn(Math.toRadians(135))
                    .forward(19)
                    //.addTemporalMarker(3,()->{

                    // })
                    .addDisplacementMarker(()->{
                        claw.setPosition(-1); // -1 is to open the claw
                    }) // pre-loaded block
                    .build();
            TrajectorySequence Left1 = drive.trajectorySequenceBuilder(new Pose2d())
                    .turn(Math.toRadians(45))
                    .lineToLinearHeading(new Pose2d(-8,-19)) //1st block
                    .addTemporalMarker(x,()->{
                        Arm.setPower(0.8);
                        sleep(x);
                    })
                    .addDisplacementMarker(()->{
                        claw.setPosition(0); // 0 is to close the claw
                    })

                    .addTemporalMarker(x,()->{
                        Arm.setPower(-0.8);
                        sleep(x);
                    })
                    .lineToLinearHeading(new Pose2d(0,0,Math.toRadians(-45)))
                    .build();

            TrajectorySequence Left2 = drive.trajectorySequenceBuilder(new Pose2d())
                    .addDisplacementMarker(()->{
                        claw.setPosition(-1);
                    })
                    .addTemporalMarker(x,()->{
                        Arm.setPower(0.8);
                        sleep(x);
                    })
                    .lineToLinearHeading(new Pose2d(0,-19,Math.toRadians(45))) //2nd block
                    .addDisplacementMarker(()->{
                        claw.setPosition(0);
                    })
                    .addTemporalMarker(x,()->{
                        Arm.setPower(-0.8); // -0.8 is for deposit position
                        sleep(x);
                    })
                    .lineToLinearHeading(new Pose2d(0,0,Math.toRadians(-45)))
                    .build();
            TrajectorySequence Left3 = drive.trajectorySequenceBuilder(new Pose2d())
                    .addDisplacementMarker(()->{
                        claw.setPosition(-1);
                    })
                    .addTemporalMarker(x,()->{
                        Arm.setPower(0.8); //0.8 is for intaking position
                        sleep(x);
                    })
                    .lineToLinearHeading(new Pose2d(10.5,-29)) //3rd block
                    .addDisplacementMarker(()->{
                        claw.setPosition(0);
                    })
                    .addTemporalMarker(x,()->{
                        Arm.setPower(-0.8);
                        sleep(x);
                    })
                    .lineToLinearHeading(new Pose2d(0,0,Math.toRadians(-45)))
                    .addDisplacementMarker(()->{
                        claw.setPosition(-1);
                    })
                    .turn(Math.toRadians(45))
                    .strafeLeft(x) //strafe to parking
                    .build();







            drive.followTrajectorySequence(Left);
            drive.followTrajectorySequence(Left1);
            drive.followTrajectorySequence(Left2);
            drive.followTrajectorySequence(Left3);
        }
    }
}
