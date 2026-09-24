package org.firstinspires.ftc.teamcode.emulator;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.drivetrain.Drivetrain;

import java.util.HashMap;
import java.util.Map;

import emulator.hardware.SimMotor;

/** Bridges Pedro Pathing drive commands to the emulator's four simulated motors. */
final class EmulatorMecanumDrive implements Drivetrain {
    private final SimMotor frontLeft;
    private final SimMotor frontRight;
    private final SimMotor backLeft;
    private final SimMotor backRight;

    private DrivePowers lastPowers = DrivePowers.zero();
    private double powerScale = 1.0;

    EmulatorMecanumDrive(
            SimMotor frontLeft,
            SimMotor frontRight,
            SimMotor backLeft,
            SimMotor backRight
    ) {
        this.frontLeft = frontLeft;
        this.frontRight = frontRight;
        this.backLeft = backLeft;
        this.backRight = backRight;
    }

    @Override
    public void drive(DrivePowers powers, boolean manual) {
        lastPowers = powers;

        // This is the same wheel mixing used by Pedro's REV-hub Mecanum drivetrain.
        double fl = powers.forward() - powers.strafe() - powers.turn();
        double fr = powers.forward() + powers.strafe() + powers.turn();
        double bl = powers.forward() + powers.strafe() - powers.turn();
        double br = powers.forward() - powers.strafe() + powers.turn();

        double max = Math.max(1.0, Math.max(
                Math.max(Math.abs(fl), Math.abs(fr)),
                Math.max(Math.abs(bl), Math.abs(br))
        ));
        powerScale = 1.0 / max;

        // SimMotor velocities are wheel velocities, so these are intentionally logical wheel
        // powers. The mirrored physical mounting represented by the real motor directions is
        // already implicit in the mecanum physics model.
        frontLeft.setPower(fl / max);
        frontRight.setPower(fr / max);
        backLeft.setPower(bl / max);
        backRight.setPower(br / max);
    }

    @Override
    public double maxScaling(DrivePowers current, DrivePowers delta) {
        double[] currentWheels = wheelPowers(current);
        double[] deltaWheels = wheelPowers(delta);
        double lambda = 1.0;

        for (int i = 0; i < currentWheels.length; i++) {
            double a = currentWheels[i];
            double b = deltaWheels[i];
            if (Math.abs(b) < 1e-9) {
                continue;
            }

            double positiveLimit = (1.0 - a) / b;
            double negativeLimit = (-1.0 - a) / b;
            if (positiveLimit >= 0.0 && positiveLimit < lambda) {
                lambda = positiveLimit;
            }
            if (negativeLimit >= 0.0 && negativeLimit < lambda) {
                lambda = negativeLimit;
            }
        }

        return Math.max(0.0, Math.min(1.0, lambda));
    }

    @Override
    public void stop() {
        stop(false);
    }

    @Override
    public void stop(boolean brake) {
        lastPowers = DrivePowers.zero();
        frontLeft.setPower(0.0);
        frontRight.setPower(0.0);
        backLeft.setPower(0.0);
        backRight.setPower(0.0);
    }

    @Override
    public Map<String, Object> debug() {
        Map<String, Object> values = new HashMap<>();
        values.put("forward", lastPowers.forward());
        values.put("strafe", lastPowers.strafe());
        values.put("turn", lastPowers.turn());
        values.put("powerScale", powerScale);
        return values;
    }

    @Override
    public double interpolateVelocity(double xRadius, double yRadius, double theta) {
        return 1.0 / (
                Math.abs(Math.cos(theta)) / xRadius
                        + Math.abs(Math.sin(theta)) / yRadius
        );
    }

    private static double[] wheelPowers(DrivePowers powers) {
        return new double[]{
                powers.forward() - powers.strafe() - powers.turn(),
                powers.forward() + powers.strafe() + powers.turn(),
                powers.forward() + powers.strafe() - powers.turn(),
                powers.forward() - powers.strafe() + powers.turn()
        };
    }
}
