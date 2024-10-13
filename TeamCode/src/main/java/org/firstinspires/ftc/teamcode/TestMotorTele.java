package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class TestMotorTele extends LinearOpMode {

    private DcMotor rightLower = null;

    @Override
    public void runOpMode() throws InterruptedException {
        rightLower = hardwareMap.get(DcMotor.class, "elevatorMotor");
        rightLower.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();
        while (opModeIsActive()){
            if(gamepad1.x) {
                rightLower.setPower(1);
            }else if(gamepad1.y){
                rightLower.setPower(0);
            }else if(gamepad1.a){
                rightLower.setPower(-1);
            }else{
                rightLower.setPower(0);
            }
        }

    }
}
