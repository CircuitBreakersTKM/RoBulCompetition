package frc.robot.commands.auto_routines;

import org.photonvision.PhotonCamera;

import edu.wpi.first.wpilibj.Notifier;
import frc.robot.commands.TrackedCommand;
import frc.robot.subsystems.LaserTurretSubsystem;

public class PointAtBlobCommand extends TrackedCommand {
    private final LaserTurretSubsystem laserTurret;
    private PhotonCamera camera;

    private Notifier notifier;
    
    // Target offset from camera center (in degrees)
    // Positive values move target right/up in camera frame
    private static final double TARGET_YAW_OFFSET = -5.0; // horizontal offset
    private static final double TARGET_PITCH_OFFSET = 0.0; // vertical offset (positive if laser is below camera)
    
    private double filteredYaw = 0.0;
    private double filteredPitch = 0.0;
    private double prevFilteredYaw = 0.0;
    private double prevFilteredPitch = 0.0;

    private static final double alpha = 0.4;

    private static final double frequencyHz = 100.0;

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
        notifier.startPeriodic(1 / frequencyHz); // 10 ms loop = 100hz
    }

    
    public void loop() {
        var results = camera.getAllUnreadResults();
        if (results == null || results.isEmpty()) return;
    
        var result = results.get(results.size() - 1);
        if (!result.hasTargets()) return;
    
        var target = result.getBestTarget();
    
        double rawYaw = target.getYaw();
        double rawPitch = target.getPitch();

        double latencySec = result.metadata.getLatencyMillis() / 1000.0;
    
        // --- FILTERING ---
        filteredYaw = filteredYaw + alpha * (rawYaw - filteredYaw);
        filteredPitch = filteredPitch + alpha * (rawPitch - filteredPitch);
    
        double yawVel = (filteredYaw - prevFilteredYaw) * frequencyHz;
        double pitchVel = (filteredPitch - prevFilteredPitch) * frequencyHz;
    
        prevFilteredYaw = filteredYaw;
        prevFilteredPitch = filteredPitch;
    
        // --- PREDICTION ---
        double predictT = Math.min(0.15, latencySec + (1.0 / frequencyHz));
        double predictedYaw = filteredYaw + yawVel * predictT;
        double predictedPitch = filteredPitch + pitchVel * predictT;
    
        // --- OFFSET CORRECTION ---
        double yawErr   = predictedYaw - TARGET_YAW_OFFSET;
        double pitchErr = predictedPitch - TARGET_PITCH_OFFSET;
    
        // --- COMPUTE DESIRED ABSOLUTE ANGLES ---
        double currentYawDeg = laserTurret.getAzimuthDegrees();
        double currentPitchDeg = laserTurret.getAltitudeDegrees();
    
        double desiredYaw = currentYawDeg + yawErr;
        double desiredPitch = currentPitchDeg + pitchErr;
    
        // --- SEND TO SMARTMOTION POSITION CONTROL ---
        laserTurret.setAzimuthTarget(desiredYaw);
        laserTurret.setAltitudeTarget(desiredPitch);
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
