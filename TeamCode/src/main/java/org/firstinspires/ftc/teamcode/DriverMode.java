package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "DriverMode", group = "Driver")

public class DriverMode extends RobotHardware {

    private double deadzone = 0.1;

    @Override
    public void runOpMode() {
        Initialise();
        waitForStart();

        while (opModeIsActive()) {
            Gamepad1();
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

/*    protected void Gamepad2() {


    /*protected void Gamepad2() {
        if(gamepad2.x) rotire_perii.setPower(0.5);
        else if(gamepad2.y) rotire_perii.setPower(-0.5);
        else rotire_perii.setPower(0);

        if(gamepad2.a) cutie_perii.setPower(0.5);
        else if(gamepad2.b) cutie_perii.setPower(-0.5);
        else cutie_perii.setPower(0);
        
        /*if(gamepad2.dpad_up) glisiera.setPower(0.5);
        else if(gamepad2.dpad_down)  glisiera.setPower(-0.5);
        else glisiera.setPower(0);*/

        /*if(gamepad2.left_stick_y > deadzone) extindere_perii.setPower(Range.clip(gamepad2.left_stick_y, 0.1, 0.7));
        else if(gamepad2.left_stick_y < -deadzone) extindere_perii.setPower(Range.clip(gamepad2.left_stick_y, -0.5, -0.1));

        if(gamepad2.dpad_up) glisiera.setPower(0.5);
        else if(gamepad2.dpad_down)  glisiera.setPower(-0.5);
        else glisiera.setPower(0);

        if(gamepad2.left_stick_y > deadzone) extindere_perii.setPower(Range.clip(gamepad2.left_stick_y, 0.1, 0.7));
        else if(gamepad2.left_stick_y < -deadzone) extindere_perii.setPower(Range.clip(gamepad2.left_stick_y, -0.5, - 0.1));
        else extindere_perii.setPower(0);*/

    }
