package frc.robot.commands.camera;

import java.util.function.DoubleSupplier;

import frc.robot.commands.TrackedCommand;
import frc.robot.subsystems.CameraTowerSubsystem;

/**
 * Command that snaps the camera to specific angles using proportional speed control.
 * Takes the shortest rotational path and smoothly decelerates as it approaches the target.
 */
public class CameraSnapCommand extends TrackedCommand {
    private final CameraTowerSubsystem cameraTower;
    private final DoubleSupplier angleSupplier;
    
    private double cameraTargetAngle;
    private boolean hasTarget;
    
    private static final double ALLOWED_ERROR = 2.0; // degrees
    private static final double MAX_SPEED = 1.0;
    private static final double MIN_SPEED = 0.05; // Minimum speed to prevent stalling
    private static final double SLOWDOWN_RANGE = 25.0; // Start slowing down 25 degrees before target
    
    /**
     * Creates a new CameraSnapCommand.
     * 
     * @param cameraTower The camera tower subsystem
     * @param angleSupplier A supplier that provides the target angle in degrees.
     *                      Should be negated POV value: () -> -controller.getPOV()
     *                      Returns 1 (from -1) when not pressed.
     */
    public CameraSnapCommand(CameraTowerSubsystem cameraTower, DoubleSupplier angleSupplier) {
        this.cameraTower = cameraTower;
        this.angleSupplier = angleSupplier;
        this.hasTarget = false;
        
        addRequirements(cameraTower);
    }
    
    @Override
    public void initialize() {
        cameraTargetAngle = 0;
        hasTarget = false;
    }
    
    @Override
    public void execute() {
        double povInput = -angleSupplier.getAsDouble();
        
        // If POV pressed, update target angle
        if (povInput != 1) {
            // Normalize desired position to 0-360 range
            cameraTargetAngle = povInput % 360;
            if (cameraTargetAngle < 0) cameraTargetAngle += 360;
            hasTarget = true;
        }
        
        // If no target set yet, do nothing
        if (!hasTarget) {
            cameraTower.setSpeed(0);
            return;
        }
        
        // Get current camera angle in degrees
        double currentPosition = cameraTower.getEncoderPosition();
        
        // Scale positions so current is always at 180° to find shortest path
        double scaledCurrent = 180;
        double scaledDesired = cameraTargetAngle - currentPosition + 180;
        
        // Normalize scaled desired to 0-360 range
        scaledDesired = scaledDesired % 360;
        if (scaledDesired < 0) scaledDesired += 360;
        
        // Calculate error (shortest path)
        double error = scaledDesired - scaledCurrent;
        
        if (Math.abs(error) < ALLOWED_ERROR) {
            cameraTower.setSpeed(0);
            return;
        }
        
        // Scale speed proportionally to distance
        double speed;
        if (Math.abs(error) > SLOWDOWN_RANGE) {
            // Full speed when far from target
            speed = MAX_SPEED;
        } else {
            // Proportional speed when close to target
            speed = MIN_SPEED + (MAX_SPEED - MIN_SPEED) * (Math.abs(error) / SLOWDOWN_RANGE);
        }
        
        cameraTower.setSpeed(Math.copySign(speed, error));
    }
    
    /**
     * Sets the camera to snap to a specific angle (in degrees).
     * Useful for programmatic control instead of POV input.
     * 
     * @param angleDegrees The target angle in degrees (0-360)
     */
    public void setTargetAngle(double angleDegrees) {
        cameraTargetAngle = angleDegrees % 360;
        if (cameraTargetAngle < 0) cameraTargetAngle += 360;
        hasTarget = true;
    }
    
    /**
     * Checks if the camera has reached its target angle.
     * 
     * @return true if within error tolerance of target
     */
    public boolean isAtTarget() {
        if (!hasTarget) return true;
        
        double currentPosition = cameraTower.getEncoderPosition();
        double scaledCurrent = 180;
        double scaledDesired = cameraTargetAngle - currentPosition + 180;
        scaledDesired = scaledDesired % 360;
        if (scaledDesired < 0) scaledDesired += 360;
        double error = scaledDesired - scaledCurrent;
        
        return Math.abs(error) < ALLOWED_ERROR;
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
