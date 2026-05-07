package org.firstinspires.ftc.teamcode;


import static org.firstinspires.ftc.teamcode.PestoFTCConfig.*;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "Forward")
@Disabled
public class Forward extends LinearOpMode {
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
        robotNew.moveForward(2);

    }
}


