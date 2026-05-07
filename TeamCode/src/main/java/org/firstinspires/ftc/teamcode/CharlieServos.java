package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;


@TeleOp
public class CharlieServos extends LinearOpMode {

    CRServo turret0;
    CRServo turret1;
    Servo aim;
    DcMotor shooter0;
    DcMotor shooter1;
    DcMotor intake0;
    DcMotor intake1;
    VisionCreator visionCreator;

    @Override
    public void updateTelemetry(Telemetry telemetry) {
        telemetry.addData("servo angle", deg);
        super.updateTelemetry(telemetry);
    }

    private void turretSetPower(double power) {
        turret0.setPower(power);
        turret1.setPower(power);
    }

    double deg = 0;
    int mode = 0;

    @Override
    public void runOpMode() {

        turret0 = hardwareMap.get(CRServo.class, "servo1");
        turret1 = hardwareMap.get(CRServo.class, "servo2");
        shooter0 = hardwareMap.get(DcMotor.class, "shooter0");
        shooter1 = hardwareMap.get(DcMotor.class, "shooter1");
        intake0 = hardwareMap.get(DcMotor.class, "intake0");
        intake1 = hardwareMap.get(DcMotor.class, "intake1");
        aim = hardwareMap.get(Servo.class, "aim");

        shooter1.setDirection(DcMotor.Direction.REVERSE);
        visionCreator = new VisionCreator(hardwareMap);
        AprilTagProcessor tagProcessor = visionCreator.getTagProcessor();
        waitForStart();
        while (opModeIsActive() && !isStopRequested()) {
            updateTelemetry(telemetry);
            if (Math.abs(gamepad1.right_stick_y) > 0.3) {
                turret0.setPower(gamepad1.right_stick_y);
                turret1.setPower(gamepad1.right_stick_y);
            } else {
                turret0.setPower(0);
                turret1.setPower(0);
            }
            if (Math.abs(gamepad1.left_stick_y) > 0.3) {
                intake0.setPower(gamepad1.left_stick_y);
                intake1.setPower(gamepad1.left_stick_y);
            } else {
                intake0.setPower(0);
                intake1.setPower(0);
            }
            shooter0.setPower(gamepad1.right_trigger - gamepad1.left_trigger);
            shooter1.setPower(gamepad1.right_trigger - gamepad1.left_trigger);
//            AprilTagDetection tag = tagProcessor.getDetections().get(0);

            if (gamepad1.right_bumper) {
                while (gamepad1.right_bumper) {
                }
                mode++;
                if (mode == 3) mode = 0;
            }
            aim.setPosition(mode * 0.15);
            visionCreator = new VisionCreator(hardwareMap);
            double turn = 0.0;
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
            }
        }
    }
}



