package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class FinalTeleOp extends LinearOpMode {
    private DcMotor leftUpper = null;
    private DcMotor leftLower = null;
    private DcMotor rightUpper = null;
    private DcMotor rightLower = null;
    private DcMotor intakeOne = null;
    private DcMotor intakeTwo = null;
    private DcMotor leftLift = null;
    private DcMotor rightLift = null;
    private static double MOTOR_ADJUST = 0.75;

    @Override public void runOpMode() {
        double drive = 0;        // Desired forward power/speed (-1 to +1)
        double strafe = 0;        // Desired strafe power/speed (-1 to +1)
        double turn = 0;        // Desired turning power/speed (-1 to +1)

        leftUpper = hardwareMap.get(DcMotor.class, "leftUpper");
        rightUpper = hardwareMap.get(DcMotor.class, "rightUpper");
        leftLower = hardwareMap.get(DcMotor.class, "leftLower");
        rightLower = hardwareMap.get(DcMotor.class,"rightLower");

        intakeOne = hardwareMap.get(DcMotor.class, "intakeOne");
        intakeTwo = hardwareMap.get(DcMotor.class, "intakeTwo");
        leftLift = hardwareMap.get(DcMotor.class, "leftLift");
        rightLift = hardwareMap.get(DcMotor.class, "rightLift");

        leftUpper.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightUpper.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftLower.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightLower.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);



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
            if(gamepad2.x){
                telemetry.addData("I am hungry for blocks!",stuff);
                intakeOne.setPower((.8)); //will need to reverse depending on position of motors
                intakeTwo.setPower((-.8));
            }else if(gamepad2.y){
                telemetry.addData("Im too full!",stuff);
                intakeOne.setPower((-.8));
                intakeTwo.setPower((.8));
            }else{
                intakeOne.setPower((0));
                intakeTwo.setPower((0));
            }
        }
    }

}
