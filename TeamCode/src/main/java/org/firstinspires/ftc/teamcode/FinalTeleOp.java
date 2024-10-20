package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class FinalTeleOp extends LinearOpMode {
    private DcMotor leftUpper = null;
    private DcMotor leftLower = null;
    private DcMotor rightUpper = null;
    private DcMotor rightLower = null;
    private DcMotor armMotor = null;
    private DcMotor liftMotor = null;

    private static double MOTOR_ADJUST = 0.75;
    private static final int ARM_COUNTS_PER_MOTOR_REV = 1996;
    private static final double ARM_DEGREES_PER_COUNT = 360.0 / ARM_COUNTS_PER_MOTOR_REV;

    private double armTargetAngle = 65.0;
    private static final double ARM_MAX_ANGLE = 180.0;
    private static final double ARM_MIN_ANGLE = 0.0;


    @Override public void runOpMode() {
        double drive = 0;        // Desired forward power/speed (-1 to +1)
        double strafe = 0;        // Desired strafe power/speed (-1 to +1)
        double turn = 0;        // Desired turning power/speed (-1 to +1)

        leftUpper = hardwareMap.get(DcMotor.class, "leftUpper");
        rightUpper = hardwareMap.get(DcMotor.class, "rightUpper");
        leftLower = hardwareMap.get(DcMotor.class, "leftLower");
        rightLower = hardwareMap.get(DcMotor.class,"rightLower");
        armMotor = hardwareMap.get(DcMotor.class, "Arm");
        liftMotor = hardwareMap.get(DcMotor.class, "Lift");

        armMotor.setTargetPosition(360);


        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftUpper.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightUpper.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftLower.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightLower.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightUpper.setDirection(DcMotorSimple.Direction.FORWARD);
        rightLower.setDirection(DcMotorSimple.Direction.FORWARD);
        leftLower.setDirection(DcMotorSimple.Direction.REVERSE);
        leftUpper.setDirection(DcMotorSimple.Direction.REVERSE);
        liftMotor.setDirection(DcMotor.Direction.FORWARD);

        setArmPosition(armTargetAngle);



        waitForStart();
        double triggerPowerAdjust = 1;
        double speedAdjust = 1.4;
        int stuff = 0;
        while (opModeIsActive()) {
            double r = Math.hypot(-gamepad1.left_stick_x, gamepad1.left_stick_y);
            double robotAngle = Math.atan2(gamepad1.left_stick_y, -gamepad1.left_stick_x) - Math.PI / 4;
            double rightX = -gamepad1.right_stick_x * 0.5;
            double v1 = r * Math.cos(robotAngle) + rightX;
            double v2 = r * Math.sin(robotAngle) - rightX;
            double v3 = r * Math.sin(robotAngle) + rightX;
            double v4 = r * Math.cos(robotAngle) - rightX;

            v1 = (v1 * triggerPowerAdjust * -1) * speedAdjust;
            v2 = (v2 * triggerPowerAdjust * -1) * speedAdjust;
            v3 = (v3 * triggerPowerAdjust * -1) * speedAdjust;
            v4 = (v4 * triggerPowerAdjust * -1) * speedAdjust;
            leftUpper.setPower(v1 * 1);
            rightUpper.setPower(v2 * 1);
            leftLower.setPower(v3 * 1);
            rightLower.setPower(v4 * 1);

            //intake!!
            // ask thepi - do i need to add zero power behavior for intake motors

            if (gamepad2.left_bumper && armTargetAngle < ARM_MAX_ANGLE) {
                armTargetAngle += 2;
                sleep(50);
            }

            if (gamepad2.right_bumper && armTargetAngle > ARM_MIN_ANGLE) {
                armTargetAngle -= 2;
                sleep(50);
            }


            setArmPosition(armTargetAngle);


            double currentArmAngle = getArmAngle();


            if (currentArmAngle >= 45.0) {

                if (gamepad2.dpad_up) {
                    liftMotor.setPower(0.5);
                } else if (gamepad2.dpad_down) {
                    liftMotor.setPower(-0.5);
                } else {
                    liftMotor.setPower(0);
                }
            }
            telemetry.addData("Arm Angle", currentArmAngle);
            telemetry.addData("Lift Position", liftMotor.getCurrentPosition());
            telemetry.update();

        }
    }
    private void setArmPosition(double angle) {
        int targetPosition = (int) (angle / ARM_DEGREES_PER_COUNT);
        armMotor.setTargetPosition(targetPosition);
        armMotor.setPower(0.3); // Adjust motor power as needed
    }


    private double getArmAngle() {
        return armMotor.getCurrentPosition() * ARM_DEGREES_PER_COUNT;
    }

}
