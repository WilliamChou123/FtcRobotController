package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.PestoFTCConfig.*;

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

    public void sleep(int time) {
        try {
            Thread.sleep(time);
        } catch (InterruptedException e) {
        }
    }

    @Override

    public void runOpMode() {
        waitForStart();
        robotNew = new WCRobotMethods(hardwareMap, telemetry);
        robotNew.initImu();
        robotNew.resetIMU();
        robotNew.armTop();
        robotNew.moveForward(Afirst);
        sleep(1000);
        robotNew.rotate(Asec);
        sleep(1000);
        robotNew.moveForward(Athird);
//        for (int i = 0; i < pos.size() - 1; i++) {
//            robotNew.moveForward(pos.get(i));
//            robotNew.rotateOp(pos.get(i + 1));
//            sleep(1500);
//        }
        double lastTime = getRuntime();
        while (getRuntime() < Afour && !isStopRequested()) {
            telemetry.addData("time", getRuntime());
            telemetry.update();
            if (getRuntime() - lastTime > 2.2) {
//                robotNew.rotate(0.1);
                robotNew.Outtake.setPower(1);
                robotNew.Intake.setPower(1);
                robotNew.Middle.setPower(1);
                robotNew.Gecko.setPower(1);
//                robotNew.rotate(-0.1);
            } else {
                robotNew.Outtake.setPower(1);
            }
        }

        robotNew.Outtake.setPower(0);
        robotNew.Intake.setPower(0);
        robotNew.Middle.setPower(0);
        robotNew.Gecko.setPower(0);

        robotNew.moveForward(Afive);
        sleep(1000);
        robotNew.rotateOp(Asix);
        sleep(1000);
        lastTime = getRuntime();
        while (getRuntime() < Afour && !isStopRequested()) {

            if (getRuntime() - lastTime > Aseven) {
                robotNew.robotOrientedDrive(0, 0.4, 0);
                robotNew.Intake.setPower(0.8);
                robotNew.Gecko.setPower(0.8);
                robotNew.Middle.setPower(0.8);

            }
        }

//        robotNew.turnTo(90);

        try {
            Thread.sleep(3000);
        } catch (InterruptedException e) {
        }
        try {
            Thread.sleep(3000);
        } catch (InterruptedException e) {
        }
        robotNew.moveForward(10);

    }
}


