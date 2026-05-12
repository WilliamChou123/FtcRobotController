package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;


@TeleOp
public class CharlieVision extends LinearOpMode {
    CRServo turret1;
    CRServo turret2;
    VisionCreator visionCreator;

    private void turretSetPower(double power) {
        turret1.setPower(power);
        turret2.setPower(power);
    }

    @Override
    public void runOpMode() {
        turret1 = hardwareMap.get(CRServo.class, "servo1");
        turret2 = hardwareMap.get(CRServo.class, "servo2");
        visionCreator = new VisionCreator(hardwareMap);
        double turn = 0.5;

        visionCreator = new VisionCreator(hardwareMap);
        AprilTagProcessor tagProcessor = visionCreator.getTagProcessor();
        turretSetPower(0.5);
        waitForStart();
        turret1.getDirection();
        while (opModeIsActive() && !isStopRequested()) {

            AprilTagDetection tag = null;
            if (!tagProcessor.getDetections().isEmpty()) {
                tag = tagProcessor.getDetections().get(0);
            }

//            turn = 0.0;
            if (tag != null && tag.ftcPose != null) {
                double bearingError = -tag.ftcPose.bearing;
                turn += 0.0005 * ((bearingError > 0) ? 1 : -1);
                if (turn >= 1) turn = 1;
                if (turn <= 0) turn = 0;

            }

            turretSetPower(turn);
//            sleep(100);


            telemetry.addData("turn", turn);


        }
    }
}



