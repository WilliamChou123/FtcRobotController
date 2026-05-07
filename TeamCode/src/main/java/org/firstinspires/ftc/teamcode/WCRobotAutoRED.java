package org.firstinspires.ftc.teamcode;


import static org.firstinspires.ftc.teamcode.PestoFTCConfig.*;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "WCRobotAuto")
@Disabled

public class WCRobotAutoRED extends LinearOpMode {
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
        robotNew.moveForward(Bfirst);
        sleep(1000);
        robotNew.rotatenormal(Bsec);
        sleep(1000);
        robotNew.moveForward(Bthird);
        double lastTime = getRuntime();
        while (getRuntime() - lastTime < 8 && !isStopRequested()) {
            telemetry.addData("time", getRuntime());
            telemetry.update();
            if (getRuntime() - lastTime > 2.2) {
                robotNew.Outtake.setPower(1);
                robotNew.Intake.setPower(1);
                robotNew.Middle.setPower(1);
                robotNew.Gecko.setPower(1);
            } else {
                robotNew.Outtake.setPower(1);
            }
            sleep(10);
        }
        robotNew.Outtake.setPower(0);
        robotNew.Intake.setPower(0);
        robotNew.Middle.setPower(0);
        robotNew.Gecko.setPower(0);

//        robotNew.moveForward(Afive);
//        sleep(1000);
//        robotNew.rotateOp(Asix);
//        sleep(1000);
//        lastTime = getRuntime();
//        while (getRuntime() < Afour && !isStopRequested()) {
//
//            if (getRuntime() - lastTime > Aseven) {
//                robotNew.robotOrientedDrive(0, 0.4, 0);
//                robotNew.Intake.setPower(0.8);
//                robotNew.Gecko.setPower(0.8);
//                robotNew.Middle.setPower(0.8);
//
//            }
//        }
//
////        robotNew.turnTo(90);
//
//        try {
//            Thread.sleep(3000);
//        } catch (InterruptedException e) {
//        }
//        try {
//            Thread.sleep(3000);
//        } catch (InterruptedException e) {
//        }
//        robotNew.moveForward(10);
//
    }
}


