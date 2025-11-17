package frc.robot.commands.auto_routines;

import org.photonvision.PhotonCamera;

import edu.wpi.first.wpilibj.Notifier;
import frc.robot.commands.TrackedCommand;
import frc.robot.subsystems.LaserTurretSubsystem;

public class PointAtBlobCommand extends TrackedCommand {
    private final LaserTurretSubsystem laserTurret;
    private PhotonCamera camera;

    private Notifier notifier;
    
    // Previous error values for derivative calculation
    private double previousYawError = 0.0;
    private double previousPitchError = 0.0;
    
    // Previous target positions for velocity estimation
    private double previousYaw = 0.0;
    private double previousPitch = 0.0;
    
    // Target offset from camera center (in degrees)
    // Positive values move target right/up in camera frame
    private static final double TARGET_YAW_OFFSET = 1.0; // horizontal offset
    private static final double TARGET_PITCH_OFFSET = -6.0; // vertical offset (positive if laser is below camera)
    
    // PID gains for yaw (azimuth) and pitch (altitude)
    // TUNING GUIDE - Do in order:
    // 1. Set KD=0, KF=0. Increase KP until it moves but oscillates slightly (try 0.01, 0.015, 0.02...)
    // 2. Increase KD to dampen oscillation (try 0.002, 0.005, 0.01...)
    // 3. Increase KF to track moving targets (try 0.1, 0.2, 0.3...)
    private static final double KP = 0.024; // Start here - increase until it moves reasonably
    private static final double KD = 0.013; // Add after KP is good - stops oscillation
    private static final double KF = 0.020; // Add last - helps track moving targets
    
    // Control parameters
    private static final double TOLERANCE = 1.0; // degrees - tighter tolerance for precision
    private static final double MAX_SPEED = 1.0; // maximum motor speed for safety
    private static final double MIN_SPEED = 0.005; // minimum speed to overcome friction

    public PointAtBlobCommand(LaserTurretSubsystem laserTurret) {
        this.laserTurret = laserTurret;
        this.camera = new PhotonCamera("Arducam-b0559-1080p-swift");
        this.camera.setPipelineIndex(0);

        this.notifier = new Notifier(this::loop);

        addRequirements(laserTurret);
    }
    
    @Override
    public void initialize() {
        super.initialize();
        notifier.startPeriodic(0.01); // 10 ms loop = 100hz
        previousYawError = 0.0;
        previousPitchError = 0.0;
        previousYaw = 0.0;
        previousPitch = 0.0;
    }

    public void loop() {
        var result = camera.getLatestResult();
        
        if (result.hasTargets()) {
            var target = result.getBestTarget();
            
            // Get target position relative to camera center
            double yaw = target.getYaw();
            double pitch = target.getPitch();
            
            // Apply offsets to account for laser/camera misalignment
            double yawError = yaw - TARGET_YAW_OFFSET;
            double pitchError = pitch - TARGET_PITCH_OFFSET;
            
            // Calculate target velocity (how fast the target is moving in camera frame)
            double yawVelocity = yaw - previousYaw;
            double pitchVelocity = pitch - previousPitch;
            
            // Calculate derivative (rate of change of error)
            double yawDerivative = yawError - previousYawError;
            double pitchDerivative = pitchError - previousPitchError;
            
            // Apply deadband tolerance to prevent jitter when close to target
            if (Math.abs(yawError) < TOLERANCE) {
                yawError = 0.0;
                yawDerivative = 0.0;
            }
            if (Math.abs(pitchError) < TOLERANCE) {
                pitchError = 0.0;
                pitchDerivative = 0.0;
            }
            
            // PDF control: P corrects error, D dampens oscillation, F tracks movement
            // Feedforward matches target velocity for smooth tracking
            double azimuthSpeed = -((KP * yawError) + (KD * yawDerivative) + (KF * yawVelocity));
            double altitudeSpeed = -((KP * pitchError) + (KD * pitchDerivative) + (KF * pitchVelocity));
            
            // Store current values for next cycle
            previousYawError = yawError;
            previousPitchError = pitchError;
            previousYaw = yaw;
            previousPitch = pitch;
            
            // Apply minimum speed to overcome friction
            if (Math.abs(azimuthSpeed) > 0.001 && Math.abs(azimuthSpeed) < MIN_SPEED) {
                azimuthSpeed = Math.copySign(MIN_SPEED, azimuthSpeed);
            }
            if (Math.abs(altitudeSpeed) > 0.001 && Math.abs(altitudeSpeed) < MIN_SPEED) {
                altitudeSpeed = Math.copySign(MIN_SPEED, altitudeSpeed);
            }
            
            // Clamp to maximum speed for safety
            azimuthSpeed = Math.max(-MAX_SPEED, Math.min(MAX_SPEED, azimuthSpeed));
            altitudeSpeed = Math.max(-MAX_SPEED, Math.min(MAX_SPEED, altitudeSpeed));
            
            laserTurret.setSpeed(azimuthSpeed, altitudeSpeed);
        } else {
            laserTurret.stop();
            previousYawError = 0.0;
            previousPitchError = 0.0;
            previousYaw = 0.0;
            previousPitch = 0.0;
        }
    }

    @Override
    public void end(boolean interrupted) {
        laserTurret.stop();
        notifier.stop();
        super.end(interrupted);
    }

    @Override
    public boolean isFinished() {
        // Never finish - continuous tracking until command is interrupted
        return false;
    }
}
