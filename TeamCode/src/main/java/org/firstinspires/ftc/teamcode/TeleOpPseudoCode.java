package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
@Disabled
public class TeleOpPseudoCode extends LinearOpMode {
    @Override
    // Declare Motor and Servos
    // Declare value consts and int variables
    public void runOpMode() throws InterruptedException {
        // Call motors from hardware map

        // set initial arm position to 360 ticks (65 degrees)

        // set motor modes

        // set motor behaviors

        // set motor directions

        waitForStart();

        while(opModeIsActive()){
            // trig for drive motor controls


            if(gamepad2.a){
                // set arm degree to 90 degrees
            }else if (gamepad2.left_bumper /* and arm degree is less than the max arm degree value(180) */ ){
                // decrease arm degree by 2 ticks
            }else if (gamepad2.right_bumper /* arm degree value is greater than the min arm degree value (65) */ ){
                // increase the arm value by 2 ticks
            }


            /* if( arm angle is above 60 degrees ){
                if(gamepad2.dpad_up){
                    increase lift value by 2
                }else if(gamepad2.dpad_down){
                    decrease lift value by 2
                }else{
                    set lift power to 0
                }

            }

             */

            /*
            telemetry.addData("Arm Angle", current arm angle);
            telemetry.addData("Lift Position", lift tick value);
            telemetry.addData("Lift Angle Value", lift angle value);
            telemetry.update();
            */
        }
    }
}
