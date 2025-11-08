package frc.robot.math;

/**
 * One-dimensional rate limiter that restricts how quickly a value can change.
 * Supports separate acceleration and deceleration limits with deadband filtering.
 * Uses wall-clock time for accurate rate limiting independent of update frequency.
 */
public class RateLimiter {
    private final double accelerationLimit;   // m/s^2, how fast we can accelerate
    private final double decelerationLimit;   // m/s^2, how fast we can decelerate
    private final double ignoreIfWithin;

    private double lastValue;
    private long lastTime; // nanoseconds

    /**
     * Creates a new RateLimiter with separate acceleration and deceleration limits.
     * 
     * @param accelerationLimit Maximum rate of change when moving away from 0 (units/s²)
     * @param decelerationLimit Maximum rate of change when moving toward 0 (units/s²)
     * @param ignoreIfWithin Deadband threshold - changes smaller than this are applied immediately
     */
    public RateLimiter(double accelerationLimit, double decelerationLimit, double ignoreIfWithin) {
        this.accelerationLimit = accelerationLimit;
        this.decelerationLimit = decelerationLimit;
        this.lastValue = 0;
        this.ignoreIfWithin = ignoreIfWithin;
        this.lastTime = System.nanoTime();
    }

    /**
     * Calculates the rate-limited output value based on current input.
     * Automatically determines whether to apply acceleration or deceleration limit.
     * 
     * @param input The desired target value
     * @return The rate-limited value
     */
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

    /**
     * Resets the rate limiter to a specific value.
     * Also resets the internal timer to current time.
     * 
     * @param value The new starting value
     */
    public void reset(double value) {
        lastValue = value;
        lastTime = System.nanoTime();
    }
}