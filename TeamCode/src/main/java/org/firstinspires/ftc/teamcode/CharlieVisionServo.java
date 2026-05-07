package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;


@TeleOp
public class CharlieVisionServo extends LinearOpMode {
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
        double turn = 0.0;
        AprilTagProcessor tagProcessor = visionCreator.getTagProcessor();
        waitForStart();
        turret1.getDirection();
        while (opModeIsActive() && !isStopRequested()) {
            AprilTagDetection tag = null;
            if (!tagProcessor.getDetections().isEmpty()) {
                tag = tagProcessor.getDetections().get(0);
            }

            turn = 0.0;
            if (tag != null && tag.ftcPose != null) {
                double bearingError = -tag.ftcPose.bearing;
                turn = Range.clip(bearingError * 0.015, -0.1, 0.1);
                telemetry.addData("bearing error deg", bearingError);
                telemetry.addData("servo power", turn);
            }

            turretSetPower(turn);

            if (gamepad1.dpad_up) {
                turretSetPower(0);
            }

            telemetry.addData("power", turret1.getPower());
            telemetry.addData("turn", turn);
            telemetry.addData("tag", tag);
            telemetry.addData("detections", tagProcessor.getDetections().size());
            telemetry.update();
        }

    }
}



