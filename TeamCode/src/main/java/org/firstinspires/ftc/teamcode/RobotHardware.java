package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public abstract class RobotHardware extends LinearOpMode {


    protected DcMotor FL = null;
    protected DcMotor FR = null;
    protected DcMotor BL = null;
    protected DcMotor BR = null;
    protected ModernRoboticsI2cGyro gyro = null;

    protected void Initialise() {

        //mapare
        FL = hardwareMap.dcMotor.get("Motor Fata Stanga");
        BL = hardwareMap.dcMotor.get("Motor Spate Stanga");
        BR = hardwareMap.dcMotor.get("Motor Spate Dreapta");
        FR = hardwareMap.dcMotor.get("Motor Fata Dreapta");
        /*gyro = (ModernRoboticsI2cGyro) hardwareMap.gyroSensor.get("gyro");*/

        //directie motoare
        FL.setDirection(DcMotorSimple.Direction.FORWARD);
        BL.setDirection(DcMotorSimple.Direction.FORWARD);
        FR.setDirection(DcMotorSimple.Direction.REVERSE);
        BR.setDirection(DcMotorSimple.Direction.REVERSE);

        //setare mod
        FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //setare motor in tensiune
        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //setare putere
        FL.setPower(0);
        FR.setPower(0);
        BL.setPower(0);
        BR.setPower(0);
    }
 }
