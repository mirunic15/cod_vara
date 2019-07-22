package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;


@Autonomous(name = "AutonomousMode", group = "Autonomous")

public class AutonomousMode extends RobotHardware {

    static final int TICS_PER_CM = 67;
    static double AngleRelativ = 0;
    static float LastAngle = 0;

    @Override
    public void runOpMode() {
        Initialise();
        waitForStart();


        while (opModeIsActive()) {
            // TEST
            /*Rotire(90);
            Rotire(-90);*/
            Strafe(StrafeDirection.FORWARD, 1, 0.7);
            Strafe(StrafeDirection.BACKWARD,1, 0.7);
            Strafe(StrafeDirection.LEFT,1, 0.7);
            Strafe(StrafeDirection.RIGHT,1, 0.7);
            Strafe(StrafeDirection.FORWARDLEFT,1, 0.7);
            Strafe(StrafeDirection.BACKWARDRIGHT,1, 0.7);
            Strafe(StrafeDirection.FORWARDRIGHT,1, 0.7);
            Strafe(StrafeDirection.BACKWARDLEFT, 1, 0.7);

        }
    }

    protected void Strafe(StrafeDirection Direction, int durata, double speed){
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

    protected void Strafe2 (int angle, double speed){
        
        if (speed < 0){
            speed *= -1;
            angle += 180;
        }
        
        angle -= 90;
        
        double VectorX = Math.cos(Math.toRadians(angle));
        double VectorY = Math.sin(Math.toRadians(angle));

        double FLBRVector = Range.clip(StrafeSpeedCalc(VectorX, VectorY, true), -1, 1);
        double FRBLVector = Range.clip(StrafeSpeedCalc(VectorX, VectorY, false), -1, 1);

        double FLBRSpeed = FLBRVector * speed;
        double FRBLSpeed = FRBLVector * speed;
        
        SetMotorsPowerDiagonally(FLBRSpeed, FRBLSpeed);

    }
    
    protected double StrafeSpeedCalc (double x, double y, boolean bFLBR){
        return Range.clip(bFLBR? y + x : y - x, -1, 1);
    }

    protected void Rotire (int unghi) {
        double currentPosition = GetAngle();
        double endPosition = currentPosition + unghi;
        int stopZone = 10;

        double viteza = Math.signum(unghi) * 0.2;


        while (opModeIsActive() && (!(endPosition - stopZone < GetAngle()) && (GetAngle() < endPosition + stopZone))) {
            FL.setPower(viteza);
            BR.setPower(-viteza);
            FR.setPower(-viteza);
            BL.setPower(viteza);

            telemetry.addData("Gyro:", GetAngle());
            telemetry.addData("End Position:", endPosition);
            telemetry.update();
        }
        StopMotors();
    }

    protected float GetGlobalAngle() {
        float currentPosition = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        return currentPosition;
    }

    protected double GetAngle() {
        double detltaAngle = GetGlobalAngle() -  LastAngle;

        if (detltaAngle>180){
            detltaAngle -=360;
        } else if (detltaAngle  < -180) {
            detltaAngle += 360;
        }

        LastAngle = GetGlobalAngle();
        AngleRelativ += detltaAngle;
        return AngleRelativ;

    }

    protected void ResetAngle () {
        AngleRelativ = 0;
        LastAngle = GetGlobalAngle();
    }

}



