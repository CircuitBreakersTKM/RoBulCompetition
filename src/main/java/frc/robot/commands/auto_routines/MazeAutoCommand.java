package frc.robot.commands.auto_routines;

import org.photonvision.PhotonCamera;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.commands.TrackedCommand;
import frc.robot.subsystems.CameraTowerSubsystem;
import frc.robot.subsystems.QRDirectionSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.subsystems.network.NetworkSubsystem;
import frc.robot.commands.camera.CameraSnapCommand;

public class MazeAutoCommand extends TrackedCommand {
    private final SwerveDriveSubsystem swerveDrive;
    private final CameraSnapCommand cameraScanCommand;
    private final QRDirectionSubsystem qrSubsystem;
    
    private double driveSpeed;
    private double currentFieldHeading;

    private String lastInstruction;
    private long lastQRTimestamp = 0;
    private double lastDistance = 0;
    private int lastTicker = 0;
    private double prevDistance = 0;
    private long prevQRTimestamp = 0;
    private double estimatedSpeedCmPerMs = 0;

    private static final double SWITCH_DISTANCE_CM = 62.5;

    public MazeAutoCommand(
            SwerveDriveSubsystem swerveDrive, 
            CameraTowerSubsystem cameraTower,
            QRDirectionSubsystem qrSubsystem,
            double driveSpeed) {
        this.swerveDrive = swerveDrive;
        this.cameraScanCommand = new CameraSnapCommand(cameraTower, () -> this.currentFieldHeading);
        this.qrSubsystem = qrSubsystem;
        this.driveSpeed = driveSpeed;
        this.currentFieldHeading = 0;

        PhotonCamera cam = new PhotonCamera("Arducam-b0559-1080p-swift");
        cam.setPipelineIndex(0);
        
        addRequirements(swerveDrive, qrSubsystem);
    }

    @Override
    public void initialize() {
        NetworkSubsystem.ENABLE_QR_SCANNING.set(true);

        cameraScanCommand.schedule();
        lastInstruction = "";
        qrSubsystem.resetCounts();
        swerveDrive.skipRateLimiting = true;
        this.currentFieldHeading = 0;
        System.out.println("[Maze] Initialized with drive speed: " + driveSpeed + " m/s");
    }

    private void switchDirections() {
        String instruction = qrSubsystem.getQRDirection();
        
        lastDistance = 0;
        lastQRTimestamp = 0;
        lastTicker = 0;
        prevDistance = 0;
        prevQRTimestamp = 0;
        estimatedSpeedCmPerMs = 0;
        
        if (instruction != null && !instruction.isEmpty() && !instruction.equalsIgnoreCase(lastInstruction)) {
            if ("Right".equalsIgnoreCase(instruction)) {
                currentFieldHeading = normalizeToCardinalDirection(currentFieldHeading + 90);
                lastInstruction = instruction;
            } else if ("Left".equalsIgnoreCase(instruction)) {
                currentFieldHeading = normalizeToCardinalDirection(currentFieldHeading - 90);
                lastInstruction = instruction;
            } else if ("Finish".equalsIgnoreCase(instruction)) {
                lastInstruction = instruction;
                driveSpeed = 0;
                currentFieldHeading = 0;
            }
            
            System.out.println("[Maze] Switched to heading: " + currentFieldHeading + "° (instruction: " + instruction + ")");
        }
    }

    private double normalizeToCardinalDirection(double angle) {
        angle = angle % 360;
        if (angle < 0) angle += 360;
        int quadrant = (int) Math.round(angle / 90.0);
        quadrant = quadrant % 4;
        return quadrant * 90.0;
    }

    @Override
    public void execute() {
        double distance = qrSubsystem.getDistance();

        if (distance != 0 && qrSubsystem.getTicker() != lastTicker) {
            long currentTime = System.currentTimeMillis();
            
            // Calculate speed from previous readings
            if (lastDistance > 0 && lastQRTimestamp > 0) {
                double distanceDelta = lastDistance - distance;
                long timeDelta = currentTime - lastQRTimestamp;
                if (timeDelta > 0) {
                    estimatedSpeedCmPerMs = distanceDelta / timeDelta;
                }
            }
            
            prevDistance = lastDistance;
            prevQRTimestamp = lastQRTimestamp;
            lastDistance = distance;
            lastQRTimestamp = currentTime;
            lastTicker = qrSubsystem.getTicker();
        }

        // Estimate current distance using last known distance and calculated speed
        double estimatedCurrentDistance = lastDistance;
        if (lastDistance > 0 && estimatedSpeedCmPerMs > 0) {
            long timeSinceLastReading = System.currentTimeMillis() - lastQRTimestamp;
            estimatedCurrentDistance = lastDistance - (estimatedSpeedCmPerMs * timeSinceLastReading);
        }
        
        if (estimatedCurrentDistance > 0 && estimatedCurrentDistance < SWITCH_DISTANCE_CM) {
            switchDirections();
        }
        
        Translation2d driveVector = getDriveVectorForHeading(currentFieldHeading);
        swerveDrive.processCarteseanInput(
            driveVector,
            0,
            false,
            false
        );
    }

    private Translation2d getDriveVectorForHeading(double heading) {
        heading = normalizeToCardinalDirection(heading);
        Rotation2d rotation = Rotation2d.fromDegrees(heading);
        double xSpeed = rotation.getSin() * driveSpeed;
        double ySpeed = rotation.getCos() * driveSpeed;
        return new Translation2d(-xSpeed, ySpeed);
    }

    @Override
    public void end(boolean interrupted) {
        swerveDrive.stop();
        swerveDrive.skipRateLimiting = false;
        cameraScanCommand.cancel();
        NetworkSubsystem.ENABLE_QR_SCANNING.set(false);
    }

    @Override
    public boolean isFinished() {
        return "Finish".equalsIgnoreCase(lastInstruction);
    }
}
