package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.robot.Robot;

@TeleOp(name = "WCRobotNew", group = "Linear Opmode")
public class WCRobotNewNew extends LinearOpMode {
    WCRobotNew robotNew;

    @Override
    public void runOpMode() {
        robotNew = new WCRobotNew(hardwareMap, telemetry);

        robotNew.Outtake.setDirection(DcMotor.Direction.REVERSE);
        robotNew.initImu();

        waitForStart();
        while (opModeIsActive() && !isStopRequested()) {
            if (gamepad1.right_bumper) {
                robotNew.imu.resetYaw();
            }
            if (gamepad1.dpad_down) {
                gamepad1.rumble(100);
                robotNew.arm.setPosition(-180);
            }
            if (gamepad1.dpad_up) {
                gamepad1.rumble(100);
                robotNew.arm.setPosition(60);
            }
            if (gamepad1.right_bumper) {

                robotNew.resetIMU(Math.PI / 2);
            }
            robotNew.Outtake.setPower(-gamepad1.left_trigger * 1.5);
            robotNew.Intake.setPower(gamepad1.left_trigger * 2);
//           robotNew.fieldOrientedDrive(gamepad1);
            robotNew.fieldOrientedDrive(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x);


        }
    }
}


