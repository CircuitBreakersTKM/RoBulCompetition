package frc.robot.interfaces;

/**
 * Interface for subsystems that control motors and can be stopped.
 * Provides a standardized API for motor control across different subsystems.
 */
public interface MotorizedSubsystem {
    /**
     * Sets the speed of one or more motors in the subsystem.
     * The number and meaning of speeds depends on the implementing subsystem.
     * 
     * @param speeds Variable number of speed values (typically -1.0 to 1.0)
     */
    public void setSpeed(double... speeds);
    
    /**
     * Stops all motors in the subsystem immediately.
     */
    public void stop();
    
    /**
     * Conditionally stops all motors based on a boolean condition.
     * 
     * @param condition If true, stops all motors; if false, does nothing
     */
    public void stopIf(boolean condition);
}
