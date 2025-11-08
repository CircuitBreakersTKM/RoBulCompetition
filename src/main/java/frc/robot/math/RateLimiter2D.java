package frc.robot.math;

import edu.wpi.first.math.geometry.Translation2d;

/**
 * Two-dimensional rate limiter for velocity control of swerve drive.
 * Separately limits tangential (speed) and normal (direction) components of velocity change.
 * This allows faster acceleration in a straight line while limiting direction changes
 * that could cause wheel slip or tipping.
 */
public class RateLimiter2D {
    private final double accelerationLimit;   // m/s^2, how fast we can accelerate
    private final double decelerationLimit;   // m/s^2, how fast we can decelerate
    private final double ignoreIfWithin;      // Ignore small changes in velocity magnitude
    private static final double MIN_VELOCITY = 1E-3; // Treat as stopped if below this speed

    private Translation2d lastVelocity = new Translation2d();
    private long lastTime;

    /**
     * Constructs a RateLimiter2D.
     * 
     * @param accelerationLimit The maximum allowed rate of speed increase (m/s²).
     * @param decelerationLimit The maximum allowed rate of speed decrease and directional change (m/s²).
     * @param ignoreIfWithin Ignores changes smaller than this threshold (m/s).
     */
    public RateLimiter2D(double accelerationLimit, double decelerationLimit, double ignoreIfWithin) {
        this.accelerationLimit = accelerationLimit;
        this.decelerationLimit = decelerationLimit;
        this.ignoreIfWithin = ignoreIfWithin;
        this.lastTime = System.nanoTime();
    }

    /**
     * Calculates the rate-limited velocity based on the desired velocity and elapsed time.
     * Decomposes velocity changes into tangential (speed) and normal (direction) components,
     * applying appropriate limits to each.
     * 
     * @param desiredVelocity The target velocity vector.
     * @return The calculated rate-limited velocity vector.
     */
    public Translation2d calculate(Translation2d desiredVelocity) {
        long now = System.nanoTime();
        double dt = (now - lastTime) / 1e9; // Time in seconds
        lastTime = now;

        // The total change vector required
        Translation2d delta = desiredVelocity.minus(lastVelocity);
        double deltaMag = delta.getNorm();

        // 1. Ignore if within the threshold
        if (deltaMag < ignoreIfWithin) {
            lastVelocity = desiredVelocity;
            return lastVelocity;
        }

        // The new allowed velocity will be built from components
        Translation2d allowedDelta;

        // 2. Handle the case where the robot is starting from a stop
        double currentSpeed = lastVelocity.getNorm();
        if (currentSpeed < MIN_VELOCITY) {
            // Treat entire movement as acceleration from rest
            double maxAccelDelta = accelerationLimit * dt;
            
            if (deltaMag > maxAccelDelta) {
                allowedDelta = delta.times(maxAccelDelta / deltaMag);
            } else {
                allowedDelta = delta;
            }
        } else {
            // 3. Separate the total change vector (delta) into tangential and normal components

            // Unit vector in the direction of current movement
            Translation2d unitLastVelocity = lastVelocity.times(1.0 / currentSpeed); 

            // --- TANGENTIAL COMPONENT (Speed Change) ---
            // Projection of the total required change onto the current velocity direction
            double deltaMagTan = delta.getX() * unitLastVelocity.getX()
                                   + delta.getY() * unitLastVelocity.getY();
            
            // Determine limit based on speeding up or slowing down
            double tangentialLimit;
            if (deltaMagTan > 0) {
                // Speeding up
                tangentialLimit = accelerationLimit * dt;
            } else {
                // Slowing down/Braking
                tangentialLimit = decelerationLimit * dt;
            }

            // Clamp the tangential magnitude change
            double deltaMagTanAllowed = Math.max(-tangentialLimit, Math.min(deltaMagTan, tangentialLimit));
            
            // The allowed change vector along the current path
            Translation2d deltaTanAllowed = unitLastVelocity.times(deltaMagTanAllowed);


            // --- NORMAL COMPONENT (Direction Change) ---
            // The normal change vector is what's left after removing the tangential component
            Translation2d deltaNormal = delta.minus(unitLastVelocity.times(deltaMagTan));

            // Directional changes always use the deceleration limit
            double normalLimit = decelerationLimit * dt;
            double deltaMagNormal = deltaNormal.getNorm();
            Translation2d deltaNormalAllowed;

            if (deltaMagNormal > normalLimit) {
                // Limit the normal component by the deceleration rate
                deltaNormalAllowed = deltaNormal.times(normalLimit / deltaMagNormal);
            } else {
                deltaNormalAllowed = deltaNormal;
            }

            // The total allowed change is the sum of the two limited components
            allowedDelta = deltaTanAllowed.plus(deltaNormalAllowed);
        }

        // Update the last velocity and return
        lastVelocity = lastVelocity.plus(allowedDelta);
        return lastVelocity;
    }

    /**
     * Resets the rate limiter to a new starting velocity.
     * Also resets the internal timer to current time.
     * 
     * @param velocity The new starting velocity.
     */
    public void reset(Translation2d velocity) {
        lastVelocity = velocity;
        lastTime = System.nanoTime();
    }
}
