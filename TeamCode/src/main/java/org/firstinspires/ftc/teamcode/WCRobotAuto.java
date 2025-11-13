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
        robotNew = new WCRobotMethods(hardwareMap, telemetry);
        robotNew.moveTo(0, 10);
        robotNew.moveTo(10, 0);
        robotNew.moveTo(0, -10);
        robotNew.moveTo(-10, 0);
    }
}


