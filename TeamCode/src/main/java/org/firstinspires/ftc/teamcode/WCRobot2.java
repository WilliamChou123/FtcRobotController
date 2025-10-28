package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

@TeleOp(name = "WCRobot2", group = "Linear Opmode")
public class WCRobot2 extends LinearOpMode {
    DcMotor frontLeft;
    DcMotor backRight;
    DcMotor backLeft;
    DcMotor Outtake; //port 2 epansion
    DcMotor Intake; //port 2 epansion
    Servo arm; // name it based on what it does

    DcMotor frontRight;
    ColorSensor color1;
    DistanceSensor distance1;
    BNO055IMU imu;

    //        public void turnCCW(int deg) {
//            motorLeft.setPower(-1);
//            motorRight.setPower(1);
//            sleep((long)460 / 90 * deg);
//            turnOff();
//        }
//        public void turnCW(int deg) {
//            motorLeft.setPower(1);
//            motorRight.setPower(-1);
//            sleep((long)460 / 90 * deg);
//            turnOff();
//        }
    public void turnOff() {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backRight.setPower(0);
        backLeft.setPower(0);
    }

    public void moveForward(int steps, double speed) {
        frontLeft.setPower(speed);
        frontRight.setPower(speed);
        backLeft.setPower(speed);
        backRight.setPower(speed);
        sleep((long) steps * 100);
        turnOff();
    }

    //        public void setWheelPower(int steps, double left, double right) {
//            frontLeft.setPower(left);
//            backLeft.setPower(right);
//            sleep((long) steps * 100);
//            turnOff();
//        }
    public void addColorTelemetry() {
        telemetry.addData("Red", (color1.red()));
        telemetry.addData("Blue", (color1.blue()));
        telemetry.update();
    }

    public void moveUntilWall() {
//            while (distance1.getDistance(DistanceUnit.CM) >= 50) {
//                moveForward(1,1);
//            }
    }

    public void outtake() {
//            frontLeft.setPower(0.1);
//            frontRight.setPower(0.1);
    }

    public void intake() {
//            frontLeft.setPower(-1);
//            frontRight.setPower(-1);
    }

    //            public boolean isWallInFront() {
//              return (distance1.getDistance(DistanceUnit.CM) <= 60)
//        }
    public void fieldCentricDrive(Gamepad gamepadStick) {
        // Get heading in radians from IMU
        double botHeading = imu.getAngularOrientation(
                AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle;

        // Read joystick inputs
        double y = -gamepadStick.left_stick_y; // Forward is negative
        double x = gamepadStick.left_stick_x;
        double rx = gamepadStick.right_stick_x; // Rotation

        // Rotate joystick by robot heading
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        // Calculate powers (normalized)
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;

        // Set motor powers
        frontLeft.setPower(frontLeftPower);
        backLeft.setPower(backLeftPower);
        frontRight.setPower(frontRightPower);
        backRight.setPower(backRightPower);
    }


    public void robotOrientedDrive(Gamepad gamepadStick) {
        frontLeft.setPower(gamepadStick.left_stick_y - gamepadStick.left_stick_x - gamepadStick.right_stick_x);
        frontRight.setPower(gamepadStick.left_stick_y + gamepadStick.left_stick_x + gamepadStick.right_stick_x);
        backLeft.setPower(gamepadStick.left_stick_y + gamepadStick.left_stick_x - gamepadStick.right_stick_x);
        backRight.setPower(gamepadStick.left_stick_y - gamepadStick.left_stick_x + gamepadStick.right_stick_x);
    }

    private double error = 0.5;

    public double getHeading() {
        return imu.getAngularOrientation(
                AxesReference.INTRINSIC,
                AxesOrder.ZYX,
                AngleUnit.RADIANS
        ).firstAngle;
    }

//    public void turnCW(double targetAngle) {
//        double Kp = 0.01; // Adjust this experimentally
//        double error = targetAngle - imu.getAngularOrientation().firstAngle;
//
//        while (Math.abs(error) > 1) { // 1 degree tolerance
//            error = targetAngle - imu.getAngularOrientation().firstAngle;
//
//            double power = Kp * error;
//
//            // Limit motor power
//            power = Math.max(Math.min(power, 0.5), -0.5);
//
//            // Mecanum turn: CW
//            frontLeft.setPower(power);
//            backLeft.setPower(power);
//            frontRight.setPower(-power);
//            backRight.setPower(-power);
//
//            sleep(10); // Prevent CPU hogging
//        }
//
//        // Stop all motors
//        frontLeft.setPower(0);
//        frontRight.setPower(0);
//        backLeft.setPower(0);
//        backRight.setPower(0);
//    }

    @Override
    public void runOpMode() {
        Intake = hardwareMap.get(DcMotor.class, "intake");
        Outtake = hardwareMap.get(DcMotor.class, "outtake");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        arm = hardwareMap.get(Servo.class, "arm");
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;

        imu.initialize(parameters);

// Small pause to let IMU settle
        telemetry.addLine("Initializing IMU...");
        telemetry.update();
        sleep(1000);  // short wait
        telemetry.addLine("IMU Ready!");
        telemetry.update();


        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        Outtake.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();
        while (opModeIsActive() && !isStopRequested()) {
            if (gamepad1.dpad_left) {
                gamepad1.rumble(1000);
            }
            Outtake.setPower(-gamepad1.left_trigger * 1.5);
            Intake.setPower(gamepad1.left_trigger * 2);
            fieldCentricDrive(gamepad1);
            if (gamepad1.dpad_down) {
                gamepad1.rumble(100);
                arm.setPosition(180);
            }
            if (gamepad1.dpad_up) {
                gamepad1.rumble(100);
                arm.setPosition(0);
            }

        }
        sleep(100);
        moveForward(5, 1.0);
    }
}


