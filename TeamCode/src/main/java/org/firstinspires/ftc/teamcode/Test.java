package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import com.shprobotics.pestocore.devices.GamepadInterface;
import com.shprobotics.pestocore.drivebases.controllers.MecanumController;
import com.shprobotics.pestocore.drivebases.controllers.TeleOpController;
import com.shprobotics.pestocore.drivebases.trackers.DeterministicTracker;
import com.shprobotics.pestocore.processing.FrontalLobe;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;


@TeleOp(group = "Test")
public class Test extends LinearOpMode {
    DcMotorEx m0, m1, m2, m3, m4, m5, m6, m7;

    @Override
    public void runOpMode() {
        m0 = hardwareMap.get(DcMotorEx.class, "shooter0");
        m1 = hardwareMap.get(DcMotorEx.class, "shooter1");
        m2 = hardwareMap.get(DcMotorEx.class, "intake0");
        m3 = hardwareMap.get(DcMotorEx.class, "intake1");
        m4 = hardwareMap.get(DcMotorEx.class, "frontLeft");
        m5 = hardwareMap.get(DcMotorEx.class, "frontRight");
        m6 = hardwareMap.get(DcMotorEx.class, "backLeft");
        m7 = hardwareMap.get(DcMotorEx.class, "backRight");

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            telemetry.addData("m0", m0.getVelocity());
            telemetry.addData("m1", m1.getVelocity());
            telemetry.addData("m2", m2.getVelocity());
            telemetry.addData("m3", m3.getVelocity());
            telemetry.addData("m4", m4.getVelocity());
            telemetry.addData("m5", m5.getVelocity());
            telemetry.addData("m6", m6.getVelocity());
            telemetry.addData("m7", m7.getVelocity());
            telemetry.update();
        }
    }
}



