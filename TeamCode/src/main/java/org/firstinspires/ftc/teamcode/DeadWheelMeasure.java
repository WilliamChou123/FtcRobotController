package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "DeadWheelMeasure", group = "Linear Opmode")
public class DeadWheelMeasure extends LinearOpMode {
    WCRobotMethods robotNew;

    @Override
    public void runOpMode() {
        waitForStart();
        while (opModeIsActive() && !isStopRequested()) {
            robotNew = new WCRobotMethods(hardwareMap, telemetry);
            robotNew.initImu();
            telemetry.addData("Left", robotNew.deadWheelLeft.getCurrentPosition());
            telemetry.addData("Right", robotNew.deadWheelRight.getCurrentPosition());
            telemetry.addData("Center", robotNew.deadWheelCenter.getCurrentPosition());
            telemetry.update();
        }
    }

}
