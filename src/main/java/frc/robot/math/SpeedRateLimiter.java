package frc.robot.math;

public class SpeedRateLimiter {
    private final double accelerationLimit;   // units per second
    private final double decelerationLimit;   // units per second

    private double lastValue;
    private long lastTime; // nanoseconds

    /**
     * @param accelerationLimit max rate when moving away from 0
     * @param decelerationLimit max rate when moving toward 0
     * @param initialValue starting value
     */
    public SpeedRateLimiter(double accelerationLimit, double decelerationLimit, double initialValue) {
        this.accelerationLimit = accelerationLimit;
        this.decelerationLimit = decelerationLimit;
        this.lastValue = initialValue;
        this.lastTime = System.nanoTime();
    }

    public double calculate(double input) {
        long now = System.nanoTime();
        double dt = (now - lastTime) / 1e9; // seconds since last update
        lastTime = now;

        double delta = input - lastValue;

        // Determine if we are accelerating (moving away from 0) or decelerating (moving toward 0)
        boolean accelerating = Math.abs(input) > Math.abs(lastValue);

        if (accelerating) {
            delta = Math.copySign(Math.min(Math.abs(delta), accelerationLimit * dt), delta);
        } else {
            delta = Math.copySign(Math.min(Math.abs(delta), decelerationLimit * dt), delta);
        }

        lastValue += delta;
        return lastValue;
    }

    public void reset(double value) {
        lastValue = value;
        lastTime = System.nanoTime();
    }
}