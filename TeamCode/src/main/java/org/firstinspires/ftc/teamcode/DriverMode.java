package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "DriverMode", group = "Driver")

public class DriverMode extends RobotHardware {

    private double deadzone = 0.1;
    private int LimitaL = 2103;
    private int LimitaR = 1989;
    private boolean Constraints = false;
    private int ZERO = 0;

    @Override
    public void runOpMode() {
        Initialise();
        waitForStart();

        while (opModeIsActive()) {
            Gamepad1();
            Gamepad2();

            telemetry.addData("Extindere Glisiere Positie: ", ExtindereGlisiere.getCurrentPosition());
            telemetry.addData("Glisiera Stanga: ", GlisieraL.getCurrentPosition());
            telemetry.addData("Glisiera Dreapta: ", GlisieraR.getCurrentPosition());
            telemetry.update();
        }
    }

    protected void Gamepad1() {
        // Ambele Joystickuri??
        // TODO Testeaza rotatia
        if (Math.abs(gamepad1.left_stick_x) > deadzone || Math.abs(gamepad1.left_stick_y) > deadzone || Math.abs(gamepad1.right_stick_x) > deadzone) {
            FL.setPower(Range.clip(gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x, -0.7, 0.7));
            FR.setPower(Range.clip(gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x, -0.7, 0.7));
            BL.setPower(Range.clip(gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.right_stick_x, -0.7, 0.7));
            BR.setPower(Range.clip(gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x, -0.7, 0.7));
        } else {
            StopMotors();
        }

    }


    protected void Gamepad2() {

        if (gamepad2.left_trigger > 0) {
            ExtindereGlisiere.setPower(0.8);
        } else if (gamepad2.right_trigger > 0) {
            ExtindereGlisiere.setPower(-0.8);
        } else {
            ExtindereGlisiere.setPower(0);
        }

        if (Constraints == false) {
            if (gamepad2.left_bumper) {
                GlisieraL.setPower(0.3);
                GlisieraR.setPower(0.3);
            } else if (gamepad2.right_bumper) {
                GlisieraL.setPower(-0.3);
                GlisieraR.setPower(-0.3);
            } else {
                GlisieraL.setPower(0);
                GlisieraR.setPower(0);
            }
        } else {
            if (gamepad2.left_bumper && GlisieraL.getCurrentPosition() < LimitaL && GlisieraR.getCurrentPosition() < LimitaR) {
                GlisieraL.setPower(0.3);
                GlisieraR.setPower(0.3);
            } else if (gamepad2.right_bumper && GlisieraL.getCurrentPosition() > ZERO && GlisieraR.getCurrentPosition() > ZERO) {
                GlisieraL.setPower(-0.3);
                GlisieraR.setPower(-0.3);
            } else {
                GlisieraL.setPower(0);
                GlisieraR.setPower(0);
            }
        }

        if (gamepad2.dpad_up) {
            Constraints = !Constraints;
        }

        if (gamepad2.a) {
            ServoCastron.setPosition(0.0);
        } else if (gamepad2.b) {
            ServoCastron.setPosition(0.5);
        } else if (gamepad2.y) {
            ServoCastron.setPosition(1.0);
        }

    }

}