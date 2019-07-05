package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;


@Autonomous(name = "AutonomousMode", group = "Autonomous")

public class AutonomousMode extends RobotHardware {

    static final int TICS_PER_CM = 67;

    @Override
    public void runOpMode() {
        Initialise();
        waitForStart();


        while (opModeIsActive()) {
            // TEST
            /*Rotire(90);
            Rotire(-90);*/
            Strafe(StrafeDirection.FORWARD, 1, 0.5);
            Strafe(StrafeDirection.BACKWARD,1, 0.5);
            Strafe(StrafeDirection.LEFT,1, 0.7);
            Strafe(StrafeDirection.RIGHT,1, 0.7);
            Strafe(StrafeDirection.FORWARDLEFT,1, 0.7);
            Strafe(StrafeDirection.BACKWARDRIGHT,1, 0.7);
            Strafe(StrafeDirection.FORWARDRIGHT,1, 0.7);
            Strafe(StrafeDirection.BACKWARDLEFT, 1, 0.7);

        }
    }

    private void Strafe(StrafeDirection Direction, int durata, double speed){
        if (Direction == StrafeDirection.BACKWARD){
            SetMotorsPower(speed, speed, speed, speed);
            sleep(durata * 1000);
        }else if (Direction == StrafeDirection.FORWARD){
            SetMotorsPower(-speed, -speed, -speed, -speed);
            sleep(durata * 1000);
        }else if (Direction == StrafeDirection.RIGHT) {
            SetMotorsPower(-speed, speed, speed, -speed);
            sleep(durata * 1000);
        }else if (Direction == StrafeDirection.LEFT){
            SetMotorsPower(speed, -speed, -speed, speed);
            sleep(durata * 1000);
        }else if (Direction == StrafeDirection.BACKWARDLEFT) {
            SetMotorsPower(speed, 0, 0, speed);
            sleep(durata * 1000);
        }else if (Direction == StrafeDirection.BACKWARDRIGHT) {
            SetMotorsPower(0, speed, speed, 0);
            sleep(durata * 1000);
        }else if (Direction == StrafeDirection.FORWARDLEFT){
            SetMotorsPower(-speed, 0, 0, -speed);
            sleep(durata * 1000);
        }else if (Direction == StrafeDirection.FORWARDRIGHT) {
            SetMotorsPower(0, -speed, -speed, 0);
            sleep(durata * 1000);
        }
       StopMotors();
    }

 protected void Rotire (int unghi) {
     float currentPosition = GetGlobalAngle();
        float endPosition = currentPosition + unghi;
        int stopZone = 10;

        double viteza = Math.signum(unghi)*0.5;

        if (endPosition>360){
            endPosition -=360;
        } else if (endPosition  < 0) {
            endPosition += 360;
        }

        while (!(endPosition - stopZone < GetGlobalAngle()) && (GetGlobalAngle() < endPosition + stopZone)){
                FL.setPower(viteza);
                BR.setPower(-viteza);
                FR.setPower(-viteza);
                BL.setPower(viteza);
            }
        StopMotors();

        }

        protected float GetGlobalAngle() {
            float currentPosition = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
            return currentPosition;
        }

    }



