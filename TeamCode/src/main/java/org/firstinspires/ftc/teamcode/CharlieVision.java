package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;


@TeleOp
public class CharlieVision extends LinearOpMode {
    DcMotor frontLeft;
    DcMotor backRight;
    DcMotor backLeft;
    DcMotor frontRight;
    DcMotor outtake;
//    DcMotor turret;

    public void robotOrientedDrive(Gamepad gamepadStick) {

        frontLeft.setPower(gamepadStick.left_stick_y - gamepadStick.left_stick_x - gamepadStick.right_stick_x);
        frontRight.setPower(gamepadStick.left_stick_y + gamepadStick.left_stick_x + gamepadStick.right_stick_x);
        backLeft.setPower(gamepadStick.left_stick_y + gamepadStick.left_stick_x - gamepadStick.right_stick_x);
        backRight.setPower(gamepadStick.left_stick_y - gamepadStick.left_stick_x + gamepadStick.right_stick_x);
    }

    @Override
    public void runOpMode() {
//            outtake = hardwareMap.get(DcMotor.class, "spinner");
//            frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
//            frontRight = hardwareMap.get(DcMotor.class, "frontRight");
//            backLeft = hardwareMap.get(DcMotor.class, "backLeft");
//            backRight = hardwareMap.get(DcMotor.class, "backRight");
//            frontLeft.setDirection(DcMotor.Direction.REVERSE);
//            backLeft.setDirection(DcMotor.Direction.REVERSE);
//        turret = hardwareMap.get(DcMotor.class, "turret");
        AprilTagLibrary myAprilTagLibrary;

        AprilTagLibrary.Builder myAprilTagLibraryBuilder;
        myAprilTagLibraryBuilder = new AprilTagLibrary.Builder();
        myAprilTagLibraryBuilder.addTag(20, "Our Awesome Team Tag", 3.5, DistanceUnit.INCH);
        myAprilTagLibraryBuilder.addTag(24, "Our 2 Team Tag", 3.5, DistanceUnit.INCH);
        myAprilTagLibrary = myAprilTagLibraryBuilder.build();

        AprilTagProcessor tagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setLensIntrinsics(950.241, 950.241, 292.042, 289.164)
                .setDrawTagOutline(true)
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .setTagLibrary(myAprilTagLibrary)
                .build();
        VisionPortal visionPortal = new VisionPortal.Builder()
                .addProcessor(tagProcessor)
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640, 480))

                .build();
        waitForStart();
        while (opModeIsActive() && !isStopRequested()) {
            sleep(10);
            if (!tagProcessor.getDetections().isEmpty()) {
                AprilTagDetection tag = tagProcessor.getDetections().get(0);

                if (tag.ftcPose != null) {
                    double bearingError = -tag.ftcPose.bearing;

                    if (Math.abs(bearingError) > 2) {
                        double turn = Range.clip(bearingError * 0.02, -0.4, 0.4);
//                        turret.setPower(turn);
                    } else {
//                        turret.setPower(0);
                    }
                } else {
//                    turret.setPower(0);
                }
            } else {
//                turret.setPower(0);
            }
        }
    }
}



