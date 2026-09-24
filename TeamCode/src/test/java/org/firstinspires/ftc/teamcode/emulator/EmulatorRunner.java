package org.firstinspires.ftc.teamcode.emulator;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;

import emulator.hardware.HubId;
import emulator.hardware.PortId;
import emulator.hardware.PortType;
import emulator.hardware.SimDevice;
import emulator.hardware.SimImu;
import emulator.hardware.SimMotor;
import emulator.sim.BatteryModel;
import emulator.sim.MecanumGeometry;
import emulator.sim.MecanumRobot;
import emulator.ui.PortRowView;
import emulator.ui.RunnerShellAppKt;
import kotlin.Unit;

/** Click the green Run arrow on {@link #main(String[])} to launch the desktop emulator. */
public final class EmulatorRunner {
    private final List<SimMotor> motors;
    private final SimImu imu;
    private final MecanumRobot robot;
    private final BatteryModel battery = new BatteryModel();
    private final AutoPath autoPath;
    private final List<PortRowView> portRows;

    private boolean initialized;
    private boolean running;
    private boolean routineComplete;
    private Throwable crash;

    private EmulatorRunner() {
        SimMotor frontLeft = motor(0, EmulatorConstants.FRONT_LEFT_NAME);
        SimMotor frontRight = motor(1, EmulatorConstants.FRONT_RIGHT_NAME);
        SimMotor backLeft = motor(2, EmulatorConstants.BACK_LEFT_NAME);
        SimMotor backRight = motor(3, EmulatorConstants.BACK_RIGHT_NAME);
        motors = Arrays.asList(frontLeft, frontRight, backLeft, backRight);
        imu = new SimImu(
                new PortId(HubId.CONTROL, PortType.I2C, 0),
                EmulatorConstants.IMU_NAME
        );

        robot = new MecanumRobot(
                frontLeft,
                frontRight,
                backLeft,
                backRight,
                new MecanumGeometry(),
                144.0,
                null
        );

        EmulatorMecanumDrive drivetrain = new EmulatorMecanumDrive(
                frontLeft,
                frontRight,
                backLeft,
                backRight
        );
        autoPath = new AutoPath(new EmulatorLocalizer(robot), drivetrain);

        portRows = Arrays.asList(
                portRow(frontLeft),
                portRow(frontRight),
                portRow(backLeft),
                portRow(backRight),
                portRow(imu)
        );
    }

    public static void main(String[] args) {
        EmulatorRunner runner = new EmulatorRunner();
        if (args.length == 1 && "--smoke-test".equals(args[0])) {
            runner.runSmokeTest();
            return;
        }
        runner.launch();
    }

    private void runSmokeTest() {
        init();
        start();

        int ticks = 0;
        while (!routineComplete && ticks < 10_000) {
            tick(0.05);
            ticks++;
        }

        if (crash != null) {
            throw new IllegalStateException("AutoPath emulator crashed", crash);
        }
        if (!routineComplete) {
            throw new IllegalStateException("AutoPath did not finish within 500 simulated seconds");
        }

        // The real LinearOpMode keeps updating after the command sequence finishes, allowing
        // Pedro to settle/hold the final pose. Exercise that behavior here too.
        for (int i = 0; i < 40; i++) {
            tick(0.05);
        }

        com.pedropathing.math.Pose pose = autoPath.pose();
        System.out.printf(
                "AutoPath emulator smoke test passed in %d ticks at (%.3f, %.3f, %.2f deg)%n",
                ticks,
                pose.x(),
                pose.y(),
                Math.toDegrees(pose.heading())
        );
    }

    private void launch() {
        RunnerShellAppKt.runRunnerShellAndBlock(
                "FTC AutoPath Emulator",
                Collections.singletonList("AutoPath (desktop)"),
                selectedIndex -> {
                    init();
                    return Unit.INSTANCE;
                },
                () -> {
                    start();
                    return Unit.INSTANCE;
                },
                () -> {
                    stop();
                    return Unit.INSTANCE;
                },
                () -> {
                    resetField();
                    return Unit.INSTANCE;
                },
                (dtSeconds, gamepads) -> {
                    tick(dtSeconds);
                    return Unit.INSTANCE;
                },
                robot::getPose,
                () -> portRows,
                this::telemetry,
                () -> crash,
                this::status,
                battery::getVoltage,
                18.0,
                18.0
        );
    }

    private void init() {
        crash = null;
        running = false;
        routineComplete = false;
        initialized = true;
        for (SimMotor motor : motors) {
            motor.setPower(0.0);
            motor.resetEncoder();
        }
        autoPath.init();
        imu.setHeadingRad(robot.getPose().getHeadingRad());
        updateBattery();
    }

    private void start() {
        if (!initialized || crash != null) {
            return;
        }
        autoPath.start();
        routineComplete = false;
        running = true;
    }

    private void stop() {
        running = false;
        routineComplete = false;
        autoPath.stop();
    }

    private void resetField() {
        if (!initialized) {
            init();
        } else {
            autoPath.resetToStart();
        }
    }

    private void tick(double dtSeconds) {
        try {
            for (SimMotor motor : motors) {
                motor.update(dtSeconds);
            }
            robot.update(dtSeconds);
            imu.setHeadingRad(robot.getPose().getHeadingRad());

            if (running) {
                autoPath.update(dtSeconds);
                if (autoPath.isFinished()) {
                    routineComplete = true;
                }
            }

            updateBattery();
        } catch (Throwable throwable) {
            crash = throwable;
            running = false;
            autoPath.stop();
        }
    }

    private void updateBattery() {
        double currentAmps = 0.0;
        for (SimMotor motor : motors) {
            currentAmps += motor.currentDrawAmps();
        }
        battery.update(currentAmps);
    }

    private List<String> telemetry() {
        if (!initialized) {
            return Collections.singletonList("Click Init, then Start, to run AutoPath.");
        }

        List<String> lines = new ArrayList<>(autoPath.telemetryLines());
        lines.add(String.format("battery: %.2f V", battery.getVoltage()));
        return lines;
    }

    private String status() {
        if (crash != null) {
            return "State: CRASHED";
        }
        if (running) {
            return routineComplete ? "State: COMPLETE (holding)" : "State: RUNNING";
        }
        return initialized ? "State: INITIALIZED/STOPPED" : "State: NOT INITIALIZED";
    }

    private static SimMotor motor(int port, String name) {
        return new SimMotor(
                new PortId(HubId.CONTROL, PortType.MOTOR, port),
                name,
                384.5,
                435.0,
                9.2
        );
    }

    private static PortRowView portRow(SimDevice device) {
        return new PortRowView(
                device.getPort().getHub().getLabel(),
                device.getPort().getType().getLabel(),
                device.getPort().getIndex(),
                device.getName(),
                device::activitySummary
        );
    }
}
