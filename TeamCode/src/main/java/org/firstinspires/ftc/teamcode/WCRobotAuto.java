package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Autonomous(name = "WCRobotAuto")
public class WCRobotAuto extends LinearOpMode {
    WCRobotMethods robotNew;

    @Override

    public void runOpMode() {
        waitForStart();
        robotNew = new WCRobotMethods(hardwareMap, telemetry);
        robotNew.initImu();
        robotNew.resetIMU();


        robotNew.moveForward(2);
        try {
            Thread.sleep(2000);
        } catch (InterruptedException e) {
        }
        robotNew.rotate(0.21);
        robotNew.moveForward(1.1);
        double lastTime = getRuntime();
        while (getRuntime() < 25 && !isStopRequested()) {
            telemetry.addData("time", getRuntime());
            telemetry.update();
            if (getRuntime() - lastTime > 2.2) {
//                robotNew.rotate(0.1);
                robotNew.Outtake.setPower(1);
                robotNew.Intake.setPower(1);
//                robotNew.rotate(-0.1);
            } else {
                robotNew.Outtake.setPower(1);
            }
        }
        robotNew.rotateOp(1.25);

//        robotNew.turnTo(90);

//        try {
//            Thread.sleep(3000);
//        } catch (InterruptedException e) {
//        }
//        try {
//            Thread.sleep(3000);
//        } catch (InterruptedException e) {
//        }
//        robotNew.moveForward(10);

    }
}


