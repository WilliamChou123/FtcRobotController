package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@TeleOp(name = "VisionTest", group = "Vision")
public class VisionTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        WCRobotMethods robot = new WCRobotMethods(hardwareMap, telemetry);
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
                .setLensIntrinsics(887.894, 887.894, 285.829, 247.743)
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
                    double drive = 0;
                    double turn = 0;

// RANGE CONTROL
                    double rangeError = tag.ftcPose.range - PestoFTCConfig.range;
                    if (Math.abs(rangeError) > 10) {
                        drive = Range.clip(rangeError * 0.02, -0.5, 0.5);
                    }

// BEARING CONTROL
                    double bearingError = tag.ftcPose.bearing * -1;
                    if (Math.abs(bearingError) > 2) {
                        turn = Range.clip(bearingError * 0.01, -0.4, 0.4);
                    }

                    robot.robotOrientedDrive(0, drive, turn);

                }
            } else {
                robot.robotOrientedDrive(0, 0, 0);
            }
        }
    }
}
