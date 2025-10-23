package frc.robot.math;

public class RateLimiter {
    private final double accelerationLimit;   // m/s^2, how fast we can accelerate
    private final double decelerationLimit;   // m/s^2, how fast we can decelerate
    private final double ignoreIfWithin;

    private double lastValue;
    private long lastTime; // nanoseconds

    /**
     * @param accelerationLimit max rate when moving away from 0
     * @param decelerationLimit max rate when moving toward 0
     * @param initialValue starting value
     */
    public RateLimiter(double accelerationLimit, double decelerationLimit, double ignoreIfWithin) {
        this.accelerationLimit = accelerationLimit;
        this.decelerationLimit = decelerationLimit;
        this.lastValue = 0;
        this.ignoreIfWithin = ignoreIfWithin;
        this.lastTime = System.nanoTime();
    }

    public double calculate(double input) {
        long now = System.nanoTime();
        double dt = (now - lastTime) / 1e9; // seconds since last update
        lastTime = now;

        double delta = input - lastValue;

        if (Math.abs(delta) < ignoreIfWithin) {
            lastValue = input;
            return lastValue;
        }

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