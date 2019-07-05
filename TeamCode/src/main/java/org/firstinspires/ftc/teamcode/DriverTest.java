package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "DriverTest", group = "Driver")

public class DriverTest extends RobotHardware {

    @Override
    public void runOpMode(){
        Initialise();
        waitForStart();

        while (opModeIsActive()){
            if (gamepad1.a){
                FR.setPower(0.7);
                FL.setPower(0.7);
                BL.setPower(0.7);
                BR.setPower(0.7);
            } else {
                FR.setPower(0);
                FL.setPower(0);
                BL.setPower(0);
                BR.setPower(0);
            }
        }
    }
}
