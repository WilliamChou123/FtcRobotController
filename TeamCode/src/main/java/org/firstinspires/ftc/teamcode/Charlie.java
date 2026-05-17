package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import com.shprobotics.pestocore.devices.GamepadInterface;
import com.shprobotics.pestocore.devices.GamepadKey;
import com.shprobotics.pestocore.drivebases.controllers.MecanumController;
import com.shprobotics.pestocore.drivebases.controllers.TeleOpController;
import com.shprobotics.pestocore.drivebases.trackers.DeterministicTracker;
import com.shprobotics.pestocore.processing.FrontalLobe;
import com.shprobotics.pestocore.processing.MotorCortex;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;


@TeleOp(group = "Charlie")
public class Charlie extends LinearOpMode {
    Servo hood;
    Servo ballBlock;
    Servo turret0, turret1;
    DcMotorEx shooter0, shooter1;
    DcMotor intake0, intake1;
    DcMotor frontLeft, backRight, backLeft, frontRight;
    DcMotor deadWheelCenter, deadWheelRight, deadWheelLeft;
    VisionCreator visionCreator;
    AprilTagProcessor tagProcessor;
    AprilTagDetection tag;
    int hoodMode = 0;
    double shootPower = 0.6;
    private final boolean AUTO_BLOCKER = true;
    double blockerMode = 3.1;
    double startTime = getRuntime();
    double turn = 0.0;
    public MecanumController mecanumController;
    public DeterministicTracker tracker;
    public TeleOpController teleOpController;
    public GamepadInterface gamepadInterface1;
    public double turretOffset = 0;

    @Override
    public void runOpMode() {

        PestoFTCConfig.initializePinpoint = true;
        FrontalLobe.initialize(hardwareMap);
        mecanumController = (MecanumController) FrontalLobe.driveController;
        tracker = FrontalLobe.tracker;
        tracker.reset();
        teleOpController = FrontalLobe.teleOpController;
        gamepadInterface1 = new GamepadInterface(gamepad1);

        //        DeadWheels:
//        control 0 = backLeft
//            expansion 1 = backRight
//            control 1= horizontal

        //Turret
        turret0 = hardwareMap.get(Servo.class, "servo1");
        turret1 = hardwareMap.get(Servo.class, "servo2");
        //Shooter
        shooter0 = hardwareMap.get(DcMotorEx.class, "shooter0");
        shooter1 = hardwareMap.get(DcMotorEx.class, "shooter1");
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

//        deadWheelCenter = hardwareMap.get(DcMotor.class, "horizontal");
//        deadWheelRight = hardwareMap.get(DcMotor.class, "backRight");
//        deadWheelLeft = hardwareMap.get(DcMotor.class, "backLeft");

        //Servos
//        turretAngle(0);
        hood = hardwareMap.get(Servo.class, "aim");
        ballBlock = hardwareMap.get(Servo.class, "ballblock");

        //Vision
        visionCreator = new VisionCreator(hardwareMap);
        tagProcessor = visionCreator.getTagProcessor();
        teleOpController.resetIMU();
        waitForStart();
        double initX = 0;
        double initY = 0;
        double autoIntake = 0;

        while (opModeIsActive() && !isStopRequested()) {
            MotorCortex.update();

            gamepadInterface1.update();
            //Power to intake and shooter, GamepadKey.a=REVERSE
            intake0.setPower(-gamepad1.left_trigger + (gamepad1.a ? 1 : 0) - autoIntake);
            intake1.setPower(-gamepad1.left_trigger + (gamepad1.a ? 1 : 0) - autoIntake);
            shooter0.setPower(gamepad1.right_trigger * shootPower + (gamepad1.x ? 1 : 0));
            shooter1.setPower(gamepad1.right_trigger * shootPower + (gamepad1.x ? 1 : 0));

            //Auto Move Ball Blocker
            if (gamepad1.right_trigger > 0.1) {
                if (getRuntime() - startTime > 1.5) {
                    if (AUTO_BLOCKER) blockerMode = 3.1;
                }

                if (shooter0.getVelocity() <= -1240)
                    autoIntake = 1;
                else
                    autoIntake = 0;

            } else {
                startTime = getRuntime();
                if (AUTO_BLOCKER) blockerMode = 4;

                autoIntake = 0;
            }
            updateBlockerPosition();
            gamepad1.setLedColor(0, 255, 0, 1000);
            //Set Power of Outtake
            if (gamepad1.dpadUpWasReleased()) shootPower += 0.05;
            if (gamepad1.dpadDownWasReleased()) shootPower -= 0.05;

            if (gamepad1.b) {
                teleOpController.resetIMU();
                gamepad1.rumble(100);
            }
            if (gamepad1.y) {
                turretOffset = Math.toDegrees(teleOpController.getHeading());
                gamepad1.rumble(100);
            }
            //Hood Deg Adjust
            if (gamepad1.rightBumperWasReleased()) {
                hoodMode++;
                if (hoodMode == 3) hoodMode = 0;
            }
            hood.setPosition(hoodMode * 0.2 - 0.1);


            //Drive
            teleOpController.driveFieldCentric(-gamepad1.left_stick_y, gamepad1.left_stick_x, -gamepad1.right_stick_x);
//          robotOrientedDrive(-gamepad1.left_stick_y, gamepad1.left_stick_x, -gamepad1.right_stick_x);

            //record init x and y from target as x0 y0
            //x' and y' is x0-imux and y0-imuy
            // theta = tan^-1(x'/y') or pi-that

//get velocity in radians /2pi
            //IMU Turret
            double headingDeg = Math.toDegrees(teleOpController.getHeading());


            double relativeAngle =
                    AngleUnit.normalizeDegrees(headingDeg - turretOffset);
            double servoPos = (relativeAngle + 180) / 360.0;
            servoPos = Range.clip(servoPos, 0.2, 1);
            if (turretOffset != 0)
                turretAngle(servoPos);
//            turn = Range.clip((Math.toDegrees(teleOpController.getHeading()) + 180) / 360, -90, 90);
//            turretAngle(turn);


            updateTelemetry(telemetry);
        }
    }

    public double getDistanceFromTag() {
        tag = null;
        if (!tagProcessor.getDetections().isEmpty()) tag = tagProcessor.getDetections().get(0);
        if (tag != null && tag.ftcPose != null) {
            telemetry.addData("tag.ftcPose.range", tag.ftcPose.range);
            return tag.ftcPose.range;
//            double bearingError = -tag.ftcPose.bearing;
//            turn = Range.clip(bearingError * 0.015, -0.1, 0.1);
//            telemetry.addData("bearing error deg", bearingError);
//            telemetry.addData("servo power", turn);
        }
        return -1;
    }

    private void updateBlockerPosition() {
        if (AUTO_BLOCKER) {
            ballBlock.setPosition(blockerMode * 0.4 - 0.8);
        } else {
            if (gamepad1.leftBumperWasReleased()) {
                blockerMode++;
                if (blockerMode == 5) blockerMode = 3;
            }

        }
    }

    @Override
    public void updateTelemetry(Telemetry telemetry) {
        telemetry.addData("Ball blocker", blockerMode);
        telemetry.addData("Shoot Power", shootPower);
        telemetry.addData("IMU Heading (Deg)", Math.toDegrees(teleOpController.getHeading()));
        telemetry.addData("IMU Heading (turn)", turn);
        telemetry.addData("IMU Heading (ANGLE1)", turret0.getPosition());
        telemetry.addData("IMU Heading (ANGLE2)", turret1.getPosition());
        telemetry.addData("IMU Heading (ANGLE2)", turret1.getPosition());
        telemetry.addData("Y", tracker.getDeltaPosition().getY());
        telemetry.addData("X", tracker.getDeltaPosition().getX());
//        double ticksPerSecond = shooter0.getVelocity();
        double rpm = (shooter0.getVelocity() / 28.0) * 60.0;
        telemetry.addData("RPM", rpm);
        double rpm2 = (shooter1.getVelocity() / 28.0) * 60.0;
        telemetry.addData("RPM2", rpm2);
        telemetry.addData("VELOCITY1", shooter0.getVelocity());


        super.updateTelemetry(telemetry);
    }

    private void turretAngle(double angle) {
        turret0.setPosition(angle);
        turret1.setPosition(angle);
    }

    private void robotOrientedDrive(double x, double y, double r) {
        frontLeft.setPower(y + x + r);
        frontRight.setPower(y - x - r);
        backLeft.setPower(y - x + r);
        backRight.setPower(y + x - r);
    }
}



