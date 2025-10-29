package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

@TeleOp
public class BasicDrive extends LinearOpMode {
    DcMotor frontLeft;
    DcMotor backRight;
    DcMotor backLeft;
    DcMotor frontRight;


    public void robotOrientedDrive(Gamepad gamepadStick) {

        frontLeft.setPower(gamepadStick.left_stick_y - gamepadStick.left_stick_x - gamepadStick.right_stick_x);
        frontRight.setPower(gamepadStick.left_stick_y + gamepadStick.left_stick_x + gamepadStick.right_stick_x);
        backLeft.setPower(gamepadStick.left_stick_y + gamepadStick.left_stick_x - gamepadStick.right_stick_x);
        backRight.setPower(gamepadStick.left_stick_y - gamepadStick.left_stick_x + gamepadStick.right_stick_x);
    }

    @Override
    public void runOpMode() {
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        //frontRight.setDirection(DcMotor.Direction.REVERSE);
        //backRight.setDirection(DcMotor.Direction.REVERSE);
        waitForStart();
        while (opModeIsActive() && !isStopRequested()) {
            robotOrientedDrive(gamepad1);
        }

    }
}


