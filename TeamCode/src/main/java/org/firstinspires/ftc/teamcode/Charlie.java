package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;


@TeleOp
public class Charlie extends LinearOpMode {
    Servo aim;
    Servo ballBlock;
    Servo turret0, turret1;
    DcMotor shooter0, shooter1;
    DcMotor intake0, intake1;
    DcMotor frontLeft, backRight, backLeft, frontRight;
    DcMotor deadWheelCenter, deadWheelRight, deadWheelLeft;
    VisionCreator visionCreator;

    double deg = 0;
    int hoodMode = 0;
    double power = 0;

    double blockerMode = 3.1;
    double startTime = getRuntime();
    double turn = 0.0;

    @Override
    public void runOpMode() {
//        DeadWheels:
//        control 0 = backLeft
//            expansion 1 = backRight
//            control 1= horizontal
        //Turret
        turret0 = hardwareMap.get(Servo.class, "servo1");
        turret1 = hardwareMap.get(Servo.class, "servo2");
        //Shooter
        shooter0 = hardwareMap.get(DcMotor.class, "shooter0");
        shooter1 = hardwareMap.get(DcMotor.class, "shooter1");
        shooter1.setDirection(DcMotor.Direction.REVERSE);

        //Intake/transfer
        intake0 = hardwareMap.get(DcMotor.class, "intake0");
        intake1 = hardwareMap.get(DcMotor.class, "intake1");
        intake1.setDirection(DcMotorSimple.Direction.REVERSE);

        //Drive
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        frontLeft.setDirection(FORWARD);
        frontRight.setDirection(REVERSE);
        backLeft.setDirection(FORWARD);
        backRight.setDirection(REVERSE);
//
//        deadWheelCenter = hardwareMap.get(DcMotor.class, "horizontal");
//        deadWheelRight = hardwareMap.get(DcMotor.class, "backRight");
//        deadWheelLeft = hardwareMap.get(DcMotor.class, "backLeft");

        //Servos
        aim = hardwareMap.get(Servo.class, "aim");
        ballBlock = hardwareMap.get(Servo.class, "ballblock");

        //Vision
        visionCreator = new VisionCreator(hardwareMap);
        AprilTagProcessor tagProcessor = visionCreator.getTagProcessor();

        waitForStart();
        while (opModeIsActive() && !isStopRequested()) {
            updateTelemetry(telemetry);
//            if (Math.abs(gamepad1.right_stick_y) > 0.3) {
//                turret0.setPower(gamepad1.right_stick_y);
//                turret1.setPower(gamepad1.right_stick_y);
//            } else {
//                turret0.setPower(0);
//                turret1.setPower(0);
//            }

            intake0.setPower(-gamepad1.left_trigger);
            intake1.setPower(-gamepad1.left_trigger);


            shooter0.setPower(gamepad1.right_trigger * power);
            shooter1.setPower(gamepad1.right_trigger * power);
//                if (getRuntime() - startTime > 1.5) {
//                    blockerMode = 4;
//                }
//            } else {
//                startTime = getRuntime();
//                blockerMode = 3.1;

            if (gamepad1.dpad_up) {
                while (gamepad1.dpad_up) {
                }
                power += 0.05;
            }
            if (gamepad1.dpad_down) {
                while (gamepad1.dpad_down) {
                }
                power -= 0.05;
            }


            robotOrientedDrive(-gamepad1.left_stick_x, gamepad1.left_stick_y, -gamepad1.right_stick_x);

            if (gamepad1.right_bumper) {
                while (gamepad1.right_bumper) {
                }
                hoodMode++;
                if (hoodMode == 3) hoodMode = 0;
            }
            aim.setPosition(hoodMode * 0.15);
            ballBlock.setPosition(blockerMode * 0.4 - 0.8);


//            if (gamepad1.left_bumper) {
//                while (gamepad1.left_bumper) {
//                }
//                blockerMode++;
//                if (blockerMode == 5) blockerMode = 3;
//            }

            AprilTagDetection tag = null;
            if (!tagProcessor.getDetections().isEmpty()) {
                tag = tagProcessor.getDetections().get(0);
            }
            updateTelemetry(telemetry);
            turn = 0.0;
            if (tag != null && tag.ftcPose != null) {
                double bearingError = -tag.ftcPose.bearing;
                turn = Range.clip(bearingError * 0.015, -0.1, 0.1);
                telemetry.addData("bearing error deg", bearingError);
                telemetry.addData("servo power", turn);
            }
            turretAngle(turn);
        }
    }


    @Override
    public void updateTelemetry(Telemetry telemetry) {
        telemetry.addData("servo angle", deg);
        telemetry.addData("Ball blocker", blockerMode);
        telemetry.addData("Shoot Power", power);
        super.updateTelemetry(telemetry);
    }

    private void turretAngle(double angle) {
        turret0.setPosition(angle);
        turret1.setPosition(angle);
    }

    public void robotOrientedDrive(double x, double y, double r) {
        frontLeft.setPower(y + x + r);
        frontRight.setPower(y - x - r);
        backLeft.setPower(y - x + r);
        backRight.setPower(y + x - r);
    }
}



