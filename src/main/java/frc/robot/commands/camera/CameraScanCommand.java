package frc.robot.commands.camera;

import java.util.function.DoubleSupplier;

import frc.robot.commands.TrackedCommand;
import frc.robot.subsystems.CameraTowerSubsystem;

/**
 * Command that continuously scans the camera left and right around a center heading.
 * For example: with center heading of 90° and scan range of 15°, scans from 75° to 105°.
 */
public class CameraScanCommand extends TrackedCommand {
    private final CameraTowerSubsystem cameraTower;
    private final DoubleSupplier centerHeadingSupplier;
    private final double scanRange;
    
    private double targetAngle;
    private boolean scanningRight;
    private double lastCenterHeading;
    private boolean scanning;
    
    private static final double ALLOWED_ERROR = 2.0; // degrees
    private static final double MAX_SPEED = 0.6; // Slower for smooth scanning
    private static final double MIN_SPEED = 0.05;
    private static final double SLOWDOWN_RANGE = 10.0;
    private static final double CENTER_SHIFT_THRESHOLD = 5.0; // degrees - when to consider center has moved
    
    /**
     * Creates a new CameraScanCommand.
     * 
     * @param cameraTower The camera tower subsystem
     * @param centerHeadingSupplier Supplier for the center angle to scan around (in degrees, 0-360)
     * @param scanRange The range to scan on each side of center (in degrees)
     *                  For example, scanRange=15 means scan ±15° from center
     */
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
        // Start by scanning to the right limit
        targetAngle = normalizeAngle(centerHeading + scanRange);
        scanningRight = true;
    }
    
    @Override
    public void execute() {
        double currentPosition = cameraTower.getEncoderPosition();
        double centerHeading = normalizeAngle(-centerHeadingSupplier.getAsDouble());
        
        // Detect if center heading has shifted significantly
        double centerShift = getAngularDifference(lastCenterHeading, centerHeading);
        if (Math.abs(centerShift) > CENTER_SHIFT_THRESHOLD) {
            // Center has moved - immediately start scanning from new center
            lastCenterHeading = centerHeading;
            targetAngle = normalizeAngle(centerHeading + scanRange);
            scanningRight = true;
        }
        
        if (!scanning) {
            // Not scanning - just stay within range of center
            if (isWithinRange(currentPosition, centerHeading)) {
                // Already within range - stop moving
                cameraTower.stop();
                return;
            } else {
                // Not within range - move to center
                targetAngle = centerHeading;
            }
        } else {
            // Normal scanning behavior
            // Check if we've reached the current target
            if (isAtTarget(currentPosition, targetAngle)) {
                // Switch direction and set new target relative to current center
                if (scanningRight) {
                    // Now scan to the left limit
                    targetAngle = normalizeAngle(centerHeading - scanRange);
                    scanningRight = false;
                } else {
                    // Now scan to the right limit
                    targetAngle = normalizeAngle(centerHeading + scanRange);
                    scanningRight = true;
                }
            }
        }
        
        // Calculate movement using shortest path
        double scaledCurrent = 180;
        double scaledDesired = targetAngle - currentPosition + 180;
        scaledDesired = normalizeAngle(scaledDesired);
        
        double error = scaledDesired - scaledCurrent;
        
        // Scale speed proportionally to distance
        double speed;
        if (Math.abs(error) > SLOWDOWN_RANGE) {
            speed = MAX_SPEED;
        } else {
            speed = MIN_SPEED + (MAX_SPEED - MIN_SPEED) * (Math.abs(error) / SLOWDOWN_RANGE);
        }
        
        cameraTower.setSpeed(Math.copySign(speed, error));
    }
    
    /**
     * Checks if camera is at the target angle within error tolerance.
     */
    private boolean isAtTarget(double currentPosition, double target) {
        double scaledCurrent = 180;
        double scaledDesired = target - currentPosition + 180;
        scaledDesired = normalizeAngle(scaledDesired);
        double error = scaledDesired - scaledCurrent;
        
        return Math.abs(error) < ALLOWED_ERROR;
    }
    
    /**
     * Checks if camera is within scan range of the center heading.
     */
    private boolean isWithinRange(double currentPosition, double centerHeading) {
        double diff = Math.abs(getAngularDifference(currentPosition, centerHeading));
        return diff <= scanRange;
    }
    
    /**
     * Normalizes an angle to 0-360 range.
     */
    private double normalizeAngle(double angle) {
        angle = angle % 360;
        if (angle < 0) angle += 360;
        return angle;
    }
    
    /**
     * Gets the shortest angular difference between two angles.
     * Returns positive if angle2 is clockwise from angle1, negative if counterclockwise.
     */
    private double getAngularDifference(double angle1, double angle2) {
        double diff = angle2 - angle1;
        if (diff > 180) diff -= 360;
        if (diff < -180) diff += 360;
        return diff;
    }
    
    /**
     * Starts scanning mode. The camera will scan back and forth within the range.
     */
    public void startScanning() {
        scanning = true;
    }
    
    /**
     * Stops scanning mode. The camera will move to and stay within the center range.
     * If already within ±scanRange of center, it stops moving.
     * If outside the range, it moves to the center position.
     */
    public void stopScanning() {
        scanning = false;
    }
    
    @Override
    public void end(boolean interrupted) {
        cameraTower.stop();
    }
    
    @Override
    public boolean isFinished() {
        return false; // Runs until cancelled
    }
}
