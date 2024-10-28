package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class TeleOp extends LinearOpMode {
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
    private double liftTargetAngle = 0;
    private static final double ARM_MAX_ANGLE = 180.0;
    private static final double ARM_MIN_ANGLE = 0.0;

    @Override public void runOpMode() {
        double drive = 0;
        double strafe = 0;
        double turn = 0;

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

            // Arm Movement
            if (gamepad1.a) {
                armTargetAngle = 90; // Set target angle to 90 degrees when 'a' is pressed
                setArmPosition(armTargetAngle);
            } else if (gamepad2.left_bumper && armTargetAngle < ARM_MAX_ANGLE) {
                armTargetAngle -= 2;
                setArmPosition(armTargetAngle);
                sleep(50);
            } else if (gamepad2.right_bumper && armTargetAngle > ARM_MIN_ANGLE) {
                armTargetAngle += 2;
                setArmPosition(armTargetAngle);
                sleep(50);
            }

            double currentArmAngle = getArmAngle();

            // Lift Movement
            if (currentArmAngle >= 65.0) {
                if (gamepad2.dpad_up) {
                    liftTargetAngle += 2;
                    setLiftPosition(liftTargetAngle);
                } else if (gamepad2.dpad_down) {
                    liftTargetAngle -= 2;
                    setLiftPosition(liftTargetAngle);
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
        armMotor.setPower(0.3);
    }

    private double getArmAngle() {
        return armMotor.getCurrentPosition() * ARM_DEGREES_PER_COUNT;
    }

    private void setLiftPosition (double angle) {
        int motorTargetPos = (int) (angle / ARM_DEGREES_PER_COUNT);
        liftMotor.setTargetPosition(motorTargetPos);
        liftMotor.setPower(0.5);
    }
}
