package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.shprobotics.pestocore.drivebases.controllers.DriveController;
import com.shprobotics.pestocore.drivebases.controllers.MecanumController;
import com.shprobotics.pestocore.drivebases.controllers.TeleOpController;
import com.shprobotics.pestocore.drivebases.trackers.DeterministicTracker;
import com.shprobotics.pestocore.drivebases.trackers.ThreeWheelOdometryTracker;
import com.shprobotics.pestocore.processing.Cerebrum;
import com.shprobotics.pestocore.processing.ConfigInterface;
import com.shprobotics.pestocore.processing.FrontalLobe;
import com.shprobotics.pestocore.processing.MotorCortex;
import com.shprobotics.pestocore.processing.PestoConfig;

import java.util.ArrayList;
import java.util.Collections;

@Config
@PestoConfig()
public class PestoFTCConfig implements ConfigInterface {
    public static boolean initialized = false; // don't mess with this :O
    public static boolean initializePinpoint = true;

    // ODOMETRY

    //        DeadWheels:
//        control 0 = backLeft
//            expansion 1 = backRight
//            control 1= horizontal
    public static String leftName = "frontRight";
    public static String centerName = "backRight";
    public static String rightName = "intake1";

    public static DcMotorSimple.Direction leftDirection = DcMotorSimple.Direction.FORWARD;
    public static DcMotorSimple.Direction centerDirection = DcMotorSimple.Direction.FORWARD;
    public static DcMotorSimple.Direction rightDirection = DcMotorSimple.Direction.REVERSE;

    public static double ODOMETRY_TICKS_PER_INCH = 505.3169;
    public static double FORWARD_OFFSET = -10;
    public static double ODOMETRY_WIDTH = 9.663;

//    public static double Afirst = 2;
//    public static double Asec = 0.17;
//    public static double Athird = 0.68;
//    public static double Bfirst = 1.78;
//    public static double Bsec = 0.29;
//    public static double Bthird = 0.92;
//    public static double range = 13;

    public static void initialize(HardwareMap hardwareMap) {

        MotorCortex.initialize(hardwareMap);
        Cerebrum.initialize();

        DriveController driveController = new MecanumController(
                MotorCortex.getMotor("frontLeft"),
                MotorCortex.getMotor("frontRight"),
                MotorCortex.getMotor("backLeft"),
                MotorCortex.getMotor("backRight")
        );

        driveController.configureMotorDirections(new DcMotorSimple.Direction[]{
                DcMotorSimple.Direction.FORWARD,
                DcMotorSimple.Direction.REVERSE,
                DcMotorSimple.Direction.FORWARD,
                DcMotorSimple.Direction.REVERSE
        });

        driveController.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        if (initializePinpoint) {
            DeterministicTracker tracker = new ThreeWheelOdometryTracker.TrackerBuilder(
                    hardwareMap,
                    ODOMETRY_TICKS_PER_INCH,
                    FORWARD_OFFSET,
                    ODOMETRY_WIDTH,
                    leftName,
                    centerName,
                    rightName,
                    leftDirection,
                    centerDirection,
                    rightDirection
            )
                    .build();

            TeleOpController teleOpController = new TeleOpController(driveController, hardwareMap);
//            teleOpController.useTrackerIMU(tracker);

            teleOpController.configureIMU(new RevHubOrientationOnRobot(
                    RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                    RevHubOrientationOnRobot.UsbFacingDirection.UP
            ));

            teleOpController.setSpeedController(gamepad -> 1.0);

//            teleOpController.counteractCentripetalForce();

            FrontalLobe.teleOpController = teleOpController;
            FrontalLobe.tracker = tracker;
        }

        FrontalLobe.driveController = driveController;
    }
}