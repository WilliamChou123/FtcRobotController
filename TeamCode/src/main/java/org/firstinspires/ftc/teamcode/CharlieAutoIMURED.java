package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.Range;

import com.shprobotics.pestocore.devices.GamepadInterface;
import com.shprobotics.pestocore.drivebases.controllers.MecanumController;
import com.shprobotics.pestocore.drivebases.controllers.TeleOpController;
import com.shprobotics.pestocore.drivebases.trackers.DeterministicTracker;
import com.shprobotics.pestocore.processing.FrontalLobe;
import com.shprobotics.pestocore.processing.MotorCortex;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@Autonomous(group = "Charlie")
public class CharlieAutoIMURED extends LinearOpMode {
    Servo hood;
    Servo ballBlock;
    Servo turret0, turret1;

    DcMotorEx shooter0, shooter1;

    DcMotor intake0, intake1;
    DcMotor frontLeft, backRight, backLeft, frontRight;

    VisionCreator visionCreator;
    AprilTagProcessor tagProcessor;
    AprilTagDetection tag;

    int hoodMode = 0;
    double shootPower = 0.6;
    private final boolean AUTO_BLOCKER = true;

    double blockerMode = 3.1;
    double startTime = 0;
    double turn = 0.0;
    double autoIntake = 0;

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

        turret0 = hardwareMap.get(Servo.class, "servo1");
        turret1 = hardwareMap.get(Servo.class, "servo2");

        shooter0 = hardwareMap.get(DcMotorEx.class, "shooter0");
        shooter1 = hardwareMap.get(DcMotorEx.class, "shooter1");
        shooter1.setDirection(DcMotor.Direction.REVERSE);

        intake0 = hardwareMap.get(DcMotor.class, "intake0");
        intake1 = hardwareMap.get(DcMotor.class, "intake1");
        intake1.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        frontLeft.setDirection(FORWARD);
        frontRight.setDirection(REVERSE);
        backLeft.setDirection(FORWARD);
        backRight.setDirection(REVERSE);

        hood = hardwareMap.get(Servo.class, "aim");
        ballBlock = hardwareMap.get(Servo.class, "ballblock");

        teleOpController.resetIMU();

        waitForStart();

        if (isStopRequested()) return;

        autoIntake = 0;
        hood.setPosition(hoodMode * 0.2 - 0.1);

        double start = getRuntime();
        startTime = getRuntime();

        while (getRuntime() - startTime < 3.8 && opModeIsActive() && !isStopRequested()) {
            robotOrientedDrive(0, 0.2, 0);
        }

        robotOrientedDrive(0, 0, 0);

        startTime = getRuntime();
        start = getRuntime();

        while (getRuntime() - start < 6 && opModeIsActive() && !isStopRequested()) {
            MotorCortex.update();

            intake0.setPower(-autoIntake);
            intake1.setPower(-autoIntake);

            shooter0.setPower(0.58);
            shooter1.setPower(0.58);

            if (getRuntime() - startTime > 1.5) {
                if (AUTO_BLOCKER) blockerMode = 3.1;
            }

            if (Math.abs(shooter0.getVelocity()) >= 1200) {
                autoIntake = 1;
            } else {
                autoIntake = 0;
            }

            updateBlockerPosition();
            updateTelemetry(telemetry);
        }

        if (AUTO_BLOCKER) blockerMode = 4;

        updateBlockerPosition();

        intake0.setPower(0);
        intake1.setPower(0);
        shooter0.setPower(0);
        shooter1.setPower(0);

        holdStill(0.35);

        double startHeading = teleOpController.getHeading();

        turnToHeading(startHeading + Math.toRadians(-37), 0.3, 0.37);

        double strafeHeading = teleOpController.getHeading();

        strafeWithHeading(-0.19, 0, strafeHeading, 1.98);

        startTime = getRuntime();

        while (getRuntime() - startTime < 5 && opModeIsActive() && !isStopRequested()) {
            strafeWithHeadingSingleLoop(0, -0.2, strafeHeading);

            intake0.setPower(-0.75);
            intake1.setPower(-0.75);

            MotorCortex.update();
            updateTelemetry(telemetry);
        }

        startTime = getRuntime();

        while (getRuntime() - startTime < 3.4 && opModeIsActive() && !isStopRequested()) {
            strafeWithHeadingSingleLoop(0, 0.2, strafeHeading);

            intake0.setPower(0);
            intake1.setPower(0);

            MotorCortex.update();
            updateTelemetry(telemetry);
        }

        strafeWithHeading(0.31, 0, strafeHeading, 1.35);

        turnToHeading(strafeHeading - Math.toRadians(-20), 0.4, 0.2);

        robotOrientedDrive(0, 0, 0);

        start = getRuntime();

        while (getRuntime() - start < 7 && opModeIsActive() && !isStopRequested()) {
            MotorCortex.update();

            intake0.setPower(-autoIntake);
            intake1.setPower(-autoIntake);

            shooter0.setPower(0.58);
            shooter1.setPower(0.58);

            if (AUTO_BLOCKER) blockerMode = 3.1;

            if (Math.abs(shooter0.getVelocity()) >= 1200) {
                autoIntake = 1;
            } else {
                autoIntake = 0;
            }

            updateBlockerPosition();
            updateTelemetry(telemetry);
        }

        robotOrientedDrive(0, 0, 0);
        intake0.setPower(0);
        intake1.setPower(0);
        shooter0.setPower(0);
        shooter1.setPower(0);
    }

    private void holdStill(double seconds) {
        double start = getRuntime();

        while (getRuntime() - start < seconds && opModeIsActive() && !isStopRequested()) {
            robotOrientedDrive(0, 0, 0);
            MotorCortex.update();
            updateTelemetry(telemetry);
        }
    }

    private void strafeWithHeading(double x, double y, double targetHeading, double seconds) {
        double start = getRuntime();

        while (getRuntime() - start < seconds && opModeIsActive() && !isStopRequested()) {
            strafeWithHeadingSingleLoop(x, y, targetHeading);

            MotorCortex.update();
            updateTelemetry(telemetry);
        }

        robotOrientedDrive(0, 0, 0);
    }

    private void strafeWithHeadingSingleLoop(double x, double y, double targetHeading) {
        double kP = 1.2;

        double headingError = angleWrap(targetHeading - teleOpController.getHeading());

        double rCorrection = Range.clip(headingError * kP, -0.18, 0.18);

        turn = rCorrection;

        robotOrientedDrive(x, y, rCorrection);
    }

    private void turnToHeading(double targetHeading, double maxPower, double seconds) {
        double start = getRuntime();
        double kP = 1.5;

        while (getRuntime() - start < seconds && opModeIsActive() && !isStopRequested()) {
            double headingError = angleWrap(targetHeading - teleOpController.getHeading());

            double r = Range.clip(headingError * kP, -maxPower, maxPower);

            if (Math.abs(headingError) < Math.toRadians(1.5)) {
                r = 0;
            }

            turn = r;

            robotOrientedDrive(0, 0, r);

            MotorCortex.update();
            updateTelemetry(telemetry);
        }

        robotOrientedDrive(0, 0, 0);
    }

    private double angleWrap(double radians) {
        while (radians > Math.PI) {
            radians -= 2.0 * Math.PI;
        }

        while (radians < -Math.PI) {
            radians += 2.0 * Math.PI;
        }

        return radians;
    }

    public double getDistanceFromTag() {
        tag = null;

        if (tagProcessor != null && !tagProcessor.getDetections().isEmpty()) {
            tag = tagProcessor.getDetections().get(0);
        }

        if (tag != null && tag.ftcPose != null) {
            telemetry.addData("tag.ftcPose.range", tag.ftcPose.range);
            return tag.ftcPose.range;
        }

        return -1;
    }

    private void updateBlockerPosition() {
        ballBlock.setPosition(blockerMode * 0.4 - 0.8);
    }

    @Override
    public void updateTelemetry(Telemetry telemetry) {
        telemetry.addData("Ball blocker", blockerMode);
        telemetry.addData("Shoot Power", shootPower);

        telemetry.addData("IMU Heading Rad", teleOpController.getHeading());
        telemetry.addData("IMU Heading Deg", Math.toDegrees(teleOpController.getHeading()));
        telemetry.addData("Turn Correction", turn);

        telemetry.addData("Turret 0", turret0.getPosition());
        telemetry.addData("Turret 1", turret1.getPosition());

        telemetry.addData("Y", tracker.getDeltaPosition().getY());
        telemetry.addData("X", tracker.getDeltaPosition().getX());

        double rpm1 = (shooter0.getVelocity() / 28.0) * 60.0;
        double rpm2 = (shooter1.getVelocity() / 28.0) * 60.0;

        telemetry.addData("RPM 1", rpm1);
        telemetry.addData("RPM 2", rpm2);
        telemetry.addData("Velocity 1", shooter0.getVelocity());
        telemetry.addData("Velocity 2", shooter1.getVelocity());

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

    double getBatteryVoltage() {
        double result = Double.POSITIVE_INFINITY;

        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            double voltage = sensor.getVoltage();

            if (voltage > 0) {
                result = Math.min(result, voltage);
            }
        }

        return result;
    }
}