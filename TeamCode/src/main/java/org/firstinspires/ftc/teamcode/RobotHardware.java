package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public abstract class RobotHardware extends LinearOpMode {


    protected DcMotor FL = null;
    protected DcMotor FR = null;
    protected DcMotor BL = null;
    protected DcMotor BR = null;

    BNO055IMU gyro;
    Orientation angles;
    Acceleration gravity;

    protected DcMotor ExtindereGlisiere = null;
    protected DcMotor GlisieraL = null;
    protected DcMotor GlisieraR = null;
    protected Servo ServoCastron = null;


    protected void Initialise() {

        //mapare
        FL = hardwareMap.dcMotor.get("MotorFL");
        BL = hardwareMap.dcMotor.get("MotorBL");
        BR = hardwareMap.dcMotor.get("MotorBR");
        FR = hardwareMap.dcMotor.get("MotorFR");

        ExtindereGlisiere = hardwareMap.dcMotor.get("MotorExtindere");
        GlisieraL = hardwareMap.dcMotor.get("MotorGlisieraL");
        GlisieraR = hardwareMap.dcMotor.get("MotorGlisieraR");

        ServoCastron = hardwareMap.servo.get("ServoCastron");

        //directie motoare
        FL.setDirection(DcMotorSimple.Direction.FORWARD);
        BL.setDirection(DcMotorSimple.Direction.FORWARD);
        FR.setDirection(DcMotorSimple.Direction.REVERSE);
        BR.setDirection(DcMotorSimple.Direction.REVERSE);

        ExtindereGlisiere.setDirection(DcMotorSimple.Direction.FORWARD);
        GlisieraL.setDirection(DcMotorSimple.Direction.FORWARD);
        GlisieraR.setDirection(DcMotorSimple.Direction.REVERSE);

        //resetare encoder
        ExtindereGlisiere.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        GlisieraL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        GlisieraR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        //setare mod
        FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        ExtindereGlisiere.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        GlisieraL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        GlisieraR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //setare motor in tensiune
        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        ExtindereGlisiere.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        GlisieraL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        GlisieraR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //setare putere
        FL.setPower(0);
        FR.setPower(0);
        BL.setPower(0);
        BR.setPower(0);

        ExtindereGlisiere.setPower(0);
        GlisieraL.setPower(0);
        GlisieraR.setPower(0);



        //gyro
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        gyro = hardwareMap.get(BNO055IMU.class, "imu");
        gyro.initialize(parameters);


    }

    protected void StopMotors(){
        FL.setPower(0);
        FR.setPower(0);
        BL.setPower(0);
        BR.setPower(0);
    }

    protected void SetMotorsPower(double fl, double fr, double bl, double br) {
        FL.setPower(fl);
        FR.setPower(fr);
        BL.setPower(bl);
        BR.setPower(br);
    }

    protected  void SetMotorsPowerDiagonally(double flbr, double frbl) {
        FL.setPower(flbr);
        BR.setPower(flbr);
        FR.setPower(frbl);
        BL.setPower(frbl);
    }

 }
