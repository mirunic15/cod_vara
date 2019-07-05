package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

public class TestAutonomie extends AutonomousMode {
    @Override
    public void runOpMode() {
        Initialise();
        waitForStart();

        gyro.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        while (opModeIsActive()) {
            if (gamepad1.a) Rotire(90);
            if (gamepad1.b) Rotire(-90);
        }
    }
}
