package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
@TeleOp
public class ServoTest extends LinearOpMode {
    private Servo claw = null;
    private Servo hand = null;

    @Override
    public void runOpMode() throws InterruptedException {
        claw = hardwareMap.get(Servo.class, "claw");
        claw.setPosition(0);
        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.a) {
                claw.setPosition(1); //open
            }else if (gamepad1.b) {
                claw.setPosition(0.5); //close
            }


            if(gamepad1.x){
                hand.setPosition(1);
            }else if(gamepad1.y){
                hand.setPosition(0.5);
            }
        }
    }
}
