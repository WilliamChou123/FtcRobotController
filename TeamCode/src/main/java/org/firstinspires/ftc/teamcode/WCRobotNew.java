package org.firstinspires.ftc.teamcode;

import android.os.CountDownTimer;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.opencv.core.Mat;

@TeleOp(name = "WCRobotNew", group = "Linear Opmode")
@Disabled

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
        int slidePos = 0;
        boolean pauseSlide = false;
        while (opModeIsActive() && !isStopRequested()) {
            if (gamepad1.left_bumper) {
                robotNew.imu.resetYaw();
                robotNew.resetIMU();
            }


            telemetry.addData("pos", pos);
            telemetry.addData("lastTime", getRuntime() - lastTime);
            telemetry.addData("slidePos", slidePos);
            telemetry.addData("armROTATION", robotNew.rotation);

            robotNew.updateTelemetry(telemetry);
            if (gamepad1.right_bumper) {
                if (getRuntime() - lastTime > 2.2) {
                    robotNew.Outtake.setPower(.8);
                    robotNew.Middle.setPower(.8);
                } else {
                    robotNew.Outtake.setPower(.8);
                }
            } else {
                //no Right trigger
                robotNew.Intake.setPower(gamepad1.left_trigger - (gamepad1.right_trigger));
                robotNew.Outtake.setPower(-(gamepad1.right_trigger));
                robotNew.Gecko.setPower(-(gamepad1.right_trigger));
                robotNew.Middle.setPower(-(gamepad1.right_trigger));

                lastTime = getRuntime();
            }
            robotNew.moveMid(gamepad1.left_trigger);
//            if (slidePos >= 0 && slidePos < 1000) {
//                slidePos += Math.round(gamepad1.left_stick_y);
//            }
//            if (gamepad2.left_stick_y > 0.1) {
//                robotNew.moveSlide1(400);
//            }
////            robotNew.moveSlide1(slidePos);
//
//            if (gamepad1.right_bumper) {
//                robotNew.resetSlides();
//            }
            if (gamepad1.dpad_left) {
                while (gamepad1.dpad_left) {
                }
                robotNew.armTop();
            }
            if (gamepad1.dpad_right) {
                while (gamepad1.dpad_right) {
                }
                robotNew.armBlock();
            }

            if (gamepad1.right_trigger > 0.5) {
                robotNew.robotOrientedDrive(gamepad1.left_stick_x * 0.3, -gamepad1.left_stick_y * 0.3, gamepad1.right_stick_x * 0.3);
            } else {
                robotNew.robotOrientedDrive(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x);
            }
        }
    }
}


