package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class Test extends LinearOpMode {

    private DcMotor rightLower = null;

    @Override
    public void runOpMode() throws InterruptedException {
        rightLower = hardwareMap.get(DcMotor.class, "elevatorMotor");
        rightLower.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();
        while (opModeIsActive()){
            if(gamepad1.x) {
                rightLower.setPower(.5);
            }else if(gamepad1.y){
                rightLower.setPower(0);
            }else if(gamepad1.a){
                rightLower.setPower(-.5);
            }else{
                rightLower.setPower(0);
            }
        }

    }
}
