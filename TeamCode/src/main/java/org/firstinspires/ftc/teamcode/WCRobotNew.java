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

            robotNew.updateTelemetry(telemetry);
            if (gamepad1.right_bumper) {
                if (getRuntime() - lastTime > 2.2) {
                    robotNew.Outtake.setPower(1);
                    robotNew.Middle.setPower(1);
                } else {
                    robotNew.Outtake.setPower(1);
                }
            } else {
                //no Right trigger
                robotNew.Intake.setPower(gamepad1.left_trigger - (gamepad1.a ? 1 : 0));
                robotNew.Outtake.setPower(-(gamepad1.a ? 1 : 0));
                lastTime = getRuntime();
            }
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
            robotNew.moveMid(gamepad1.left_trigger);
            if (gamepad1.right_trigger > 0.5) {
                robotNew.robotOrientedDrive(-gamepad1.left_stick_x * 0.3, gamepad1.left_stick_y * 0.3, gamepad1.right_stick_x * 0.3);
            } else {
                robotNew.robotOrientedDrive(-gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);

            }
//          robotNew.fieldOrientedDrive(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x);


        }
    }
}


