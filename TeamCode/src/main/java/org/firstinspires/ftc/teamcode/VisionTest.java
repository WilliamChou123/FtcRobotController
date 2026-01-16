package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@TeleOp
public class VisionTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        WCRobotMethods robot = new WCRobotMethods(hardwareMap, telemetry);

        AprilTagProcessor tagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setLensIntrinsics(887.894, 887.894, 285.829, 247.743)
                .setDrawTagOutline(true)
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
                telemetry.addData("tag id", tag.id);
                telemetry.addData("tag z", tag.ftcPose.range);
                if ((double) (tag.ftcPose.range) > 30.0) {
                    robot.robotOrientedDrive(0, 0.2, 0);

                } else {
                    robot.robotOrientedDrive(0, -0.2, 0);
                }
            }
        }
    }
}
