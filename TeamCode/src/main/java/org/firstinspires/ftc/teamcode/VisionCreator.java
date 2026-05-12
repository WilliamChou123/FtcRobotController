package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

public class VisionCreator {
    AprilTagLibrary myAprilTagLibrary;
    AprilTagLibrary.Builder myAprilTagLibraryBuilder;
    AprilTagProcessor tagProcessor;
    VisionPortal visionPortal;

    public VisionCreator(HardwareMap hardwareMap) {

        myAprilTagLibraryBuilder = new AprilTagLibrary.Builder();
        myAprilTagLibraryBuilder.addTag(20, "Our Awesome Team Tag", 3.5, DistanceUnit.INCH);
        myAprilTagLibraryBuilder.addTag(24, "Our 2 Team Tag", 3.5, DistanceUnit.INCH);
        myAprilTagLibraryBuilder.addTag(22, "Our 2 Team Tag", 3.5, DistanceUnit.INCH);

        myAprilTagLibrary = myAprilTagLibraryBuilder.build();

        tagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setLensIntrinsics(950.241, 950.241, 292.042, 289.164)
                .setDrawTagOutline(true)
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .setTagLibrary(myAprilTagLibrary)
                .build();
        visionPortal = new VisionPortal.Builder()
                .addProcessor(tagProcessor)
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640, 480))
                .build();
    }

    public AprilTagProcessor getTagProcessor() {
        return tagProcessor;
    }
}