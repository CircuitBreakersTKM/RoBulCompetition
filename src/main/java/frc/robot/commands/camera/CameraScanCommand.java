package frc.robot.commands.camera;

import java.util.function.DoubleSupplier;
import frc.robot.commands.TrackedCommand;
import frc.robot.subsystems.CameraTowerSubsystem;

/**
 * Command that either scans left/right around a center heading
 * or stays near the center (no-scan mode).
 *
 * Scanning mode:
 *  - Scans between (center ± scanRange) using SCAN_SPEED.
 *
 * Non-scanning mode:
 *  - If inside range: do nothing.
 *  - If outside range: move back into range at MAX_SPEED.
 */
public class CameraScanCommand extends TrackedCommand {
    private final CameraTowerSubsystem cameraTower;
    private final DoubleSupplier centerHeadingSupplier;
    private final double scanRange;

    private double targetAngle;
    private boolean scanningRight;
    private double lastCenterHeading;
    private boolean scanning;

    // === Constants ===
    private static final double ALLOWED_ERROR = 2.0;          // degrees
    private static final double SCAN_SPEED = 0.25;            // scanning motion speed
    private static final double MAX_SPEED = 1.0;              // for moving into range fast
    private static final double MIN_SPEED = 0.05;              // minimum speed when slowing down
    private static final double SLOWDOWN_RANGE = 20.0;        // degrees - start slowing down within this range

    public CameraScanCommand(CameraTowerSubsystem cameraTower, DoubleSupplier centerHeadingSupplier, double scanRange) {
        this.cameraTower = cameraTower;
        this.centerHeadingSupplier = centerHeadingSupplier;
        this.scanRange = Math.abs(scanRange);
        this.scanningRight = true;
        this.scanning = true;
        addRequirements(cameraTower);
    }

    @Override
    public void initialize() {
        double centerHeading = normalizeAngle(-centerHeadingSupplier.getAsDouble());
        lastCenterHeading = centerHeading;
        targetAngle = normalizeAngle(centerHeading + scanRange);
        scanningRight = true;
    }

    @Override
    public void execute() {
        double currentPos = cameraTower.getEncoderPosition();
        double centerHeading = normalizeAngle(-centerHeadingSupplier.getAsDouble());

        // Update target if center heading changed
        if (Math.abs(getAngularDifference(lastCenterHeading, centerHeading)) > ALLOWED_ERROR) {
            lastCenterHeading = centerHeading;
            targetAngle = normalizeAngle(centerHeading + scanRange);
            scanningRight = true;
        }

        if (scanning) {
            handleScanMode(currentPos, centerHeading);
        } else {
            handleNoScanMode(currentPos, centerHeading);
        }
    }

    /** Scanning mode: sweep left and right using SCAN_SPEED */
    private void handleScanMode(double currentPos, double centerHeading) {
        // If outside scan range, move to range with speed ramping
        if (!isWithinRange(currentPos, centerHeading)) {
            double error = getAngularDifference(currentPos, centerHeading);
            double speed = calculateApproachSpeed(Math.abs(error));
            cameraTower.setSpeed(Math.copySign(speed, error));
            return;
        }

        // Within range and scanning - if reached one end, switch direction
        if (isAtTarget(currentPos, targetAngle)) {
            if (scanningRight) {
                targetAngle = normalizeAngle(centerHeading - scanRange);
                scanningRight = false;
            } else {
                targetAngle = normalizeAngle(centerHeading + scanRange);
                scanningRight = true;
            }
        }

        // Scan side to side using SCAN_SPEED
        double error = getAngularDifference(currentPos, targetAngle);
        cameraTower.setSpeed(Math.copySign(SCAN_SPEED, error));
    }

    /** Non-scan mode: stay within range, or zoom in if outside */
    private void handleNoScanMode(double currentPos, double centerHeading) {
        if (isWithinRange(currentPos, centerHeading)) {
            // Within range and not scanning - don't move
            cameraTower.stop();
        } else {
            // Outside range - move to center with speed ramping
            double error = getAngularDifference(currentPos, centerHeading);
            double speed = calculateApproachSpeed(Math.abs(error));
            cameraTower.setSpeed(Math.copySign(speed, error));
        }
    }

    /** Calculate speed for approaching target - ramps down to prevent oscillation */
    private double calculateApproachSpeed(double distanceToTarget) {
        if (distanceToTarget > SLOWDOWN_RANGE) {
            return MAX_SPEED;
        } else {
            // Linear ramp from MIN_SPEED to MAX_SPEED
            return MIN_SPEED + (MAX_SPEED - MIN_SPEED) * (distanceToTarget / SLOWDOWN_RANGE);
        }
    }

    private boolean isAtTarget(double currentPos, double target) {
        return Math.abs(getAngularDifference(currentPos, target)) < ALLOWED_ERROR;
    }

    private boolean isWithinRange(double currentPos, double centerHeading) {
        return Math.abs(getAngularDifference(currentPos, centerHeading)) <= scanRange;
    }

    private double normalizeAngle(double angle) {
        angle = angle % 360;
        if (angle < 0) angle += 360;
        return angle;
    }

    private double getAngularDifference(double a1, double a2) {
        double diff = a2 - a1;
        if (diff > 180) diff -= 360;
        if (diff < -180) diff += 360;
        return diff;
    }

    public void startScanning() {
        scanning = true;
    }

    public void stopScanning() {
        scanning = false;
    }

    @Override
    public void end(boolean interrupted) {
        cameraTower.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
