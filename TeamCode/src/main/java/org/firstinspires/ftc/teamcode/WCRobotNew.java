package org.firstinspires.ftc.teamcode;

import android.os.CountDownTimer;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.opencv.core.Mat;

@TeleOp(name = "WCRobotNew", group = "Linear Opmode")
public class WCRobotNew extends LinearOpMode {
    WCRobotMethods robotNew;

    @Override
    public void runOpMode() {
        robotNew = new WCRobotMethods(hardwareMap, telemetry);
        double lastTime = getRuntime();
//        robotNew.Outtake.setDirection(DcMotor.Direction.REVERSE);
        robotNew.initImu();

        waitForStart();
        int pos = 45;
        while (opModeIsActive() && !isStopRequested()) {
            if (gamepad1.right_bumper) {
                robotNew.imu.resetYaw();
                robotNew.resetIMU();
            }
            if (gamepad1.dpad_down) {
                while (gamepad1.dpad_down) {
                }
                gamepad1.rumble(100);
                pos -= 5;
                robotNew.arm.setPosition(Math.toRadians(pos));
            }
            if (gamepad1.dpad_up) {
                while (gamepad1.dpad_up) {
                }
                gamepad1.rumble(100);
                pos += 5;
                robotNew.arm.setPosition(Math.toRadians(pos));
            }


            telemetry.addData("pos", pos);
            telemetry.addData("lastTime", getRuntime() - lastTime);


            robotNew.updateTelemetry(telemetry);
            robotNew.Outtake.setPower(gamepad1.right_trigger * 0.8 - (gamepad1.a ? 1 : 0));
//            robotNew.Intake.setPower(gamepad1.left_trigger * 1 - (gamepad1.a ? 1 : 0));
            if (gamepad1.right_trigger > 0.1) {
                if (getRuntime() - lastTime > 1.5) {
                    robotNew.Outtake.setPower(1.5);
                    robotNew.Intake.setPower(2);
                } else {
                    robotNew.Outtake.setPower(gamepad1.right_trigger);
                }
            } else {
                robotNew.Intake.setPower(gamepad1.left_trigger - (gamepad1.a ? 1 : 0));
                robotNew.Outtake.setPower(-(gamepad1.a ? 1 : 0));
                lastTime = getRuntime();
            }


            robotNew.robotOrientedDrive(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x);

//            robotNew.fieldOrientedDrive(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x);


        }
    }
}


