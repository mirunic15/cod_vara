package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

@Autonomous(name = "TestAutonomie", group = "Autonomous")


public class TestAutonomie extends AutonomousMode {
    @Override
    public void runOpMode() {
        Initialise();
        waitForStart();

        gyro.startAccelerationIntegration(new Position(), new Velocity(), 1000);
        ResetAngle();

        while (opModeIsActive()) {
            if (gamepad1.a) Rotire(90);
            if (gamepad1.b) Rotire(-90);
            if (gamepad1.x) {
                /*Strafe(StrafeDirection.FORWARD, 1, 0.5);
                Strafe(StrafeDirection.BACKWARD, 1, 0.5);
                Strafe(StrafeDirection.LEFT, 1, 0.5);
                Strafe(StrafeDirection.RIGHT, 1, 0.5);
                Strafe(StrafeDirection.FORWARDLEFT, 1, 0.5);
                Strafe(StrafeDirection.FORWARDRIGHT, 1, 0.5);
                Strafe(StrafeDirection.BACKWARDLEFT, 1, 0.5);
                Strafe(StrafeDirection.BACKWARDRIGHT, 1, 0.5);*/
                Strafe2(30,0.7);
                Strafe2(-30,0.7);
                Strafe2(60,0.7);
                Strafe2(-60,0.7);
            }
        }
    }
}
