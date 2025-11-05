package frc.robot.interfaces;

public interface MotorizedSubsystem {
    public void setSpeed(double... speeds);
    public void stop();
    public void stopIf(boolean condition);
}
