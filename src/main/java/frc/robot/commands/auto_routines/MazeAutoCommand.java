package frc.robot.commands.auto_routines;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.commands.TrackedCommand;
import frc.robot.subsystems.CameraTowerSubsystem;
import frc.robot.subsystems.QRDirectionSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.commands.camera.CameraScanCommand;

public class MazeAutoCommand extends TrackedCommand {
    private final SwerveDriveSubsystem swerveDrive;
    private final CameraScanCommand cameraScanCommand;
    private final QRDirectionSubsystem qrSubsystem;
    
    private double driveSpeed;
    private double currentFieldHeading; // 0=forward, 90=right, 180=back, 270=left
    private String lastInstruction; // Track last processed instruction
    private double lastDistance = 0;

    private static final double SWITCH_DISTANCE_CM = 60.0; // Distance threshold to trigger direction switch

    /**
     * Creates a new MazeAutoCommand that drives field-relative using QR code directions
     * and rotates the camera to face forward in the drive direction.
     * 
     * Call switchDirections() to read the QR direction and update the robot's heading.
     * 
     * @param swerveDrive The swerve drive subsystem
     * @param cameraTower The camera tower subsystem
     * @param qrSubsystem The QR direction subsystem for reading directions
     * @param driveSpeed The speed at which to drive (meters per second)
     */
    public MazeAutoCommand(
            SwerveDriveSubsystem swerveDrive, 
            CameraTowerSubsystem cameraTower,
            QRDirectionSubsystem qrSubsystem,
            double driveSpeed) {
        this.swerveDrive = swerveDrive;
        this.cameraScanCommand = new CameraScanCommand(cameraTower, () -> this.currentFieldHeading, 15); // No POV input
        this.qrSubsystem = qrSubsystem;
        this.driveSpeed = driveSpeed;
        this.currentFieldHeading = 0; // Start driving forward
        
        addRequirements(swerveDrive, qrSubsystem);
    }

    @Override
    public void initialize() {
        cameraScanCommand.schedule();
        lastInstruction = ""; // No instruction processed yet
        qrSubsystem.resetCounts();
        swerveDrive.skipRateLimiting = true;
        this.currentFieldHeading = 0;

        System.out.println("[Maze] Initialized with drive speed: " + driveSpeed + " m/s");
    }

    /**
     * Reads the most common QR direction and switches the robot to that heading.
     * Call this method to trigger a direction change based on accumulated QR detections.
     * Only supports 0, 90, 180, and 270 degree headings.
     */
    private void switchDirections() {
        // Get the most common direction and reset counts
        String instruction = qrSubsystem.getQRDirection();
        
        // Only process new instructions (not empty and different from last)
        if (instruction != null && !instruction.isEmpty() && !instruction.equalsIgnoreCase(lastInstruction)) {
            if ("Right".equalsIgnoreCase(instruction)) {
                // Turn right: add 90 degrees to heading
                currentFieldHeading = normalizeToCardinalDirection(currentFieldHeading + 90);
                lastInstruction = instruction;
            } else if ("Left".equalsIgnoreCase(instruction)) {
                // Turn left: subtract 90 degrees from heading
                currentFieldHeading = normalizeToCardinalDirection(currentFieldHeading - 90);
                lastInstruction = instruction;
            } else if ("Finish".equalsIgnoreCase(instruction)) {
                // Finish command received
                lastInstruction = instruction;
                driveSpeed = 0;
                currentFieldHeading = 0; // Face forward
            }
            
            System.out.println("[Maze] Switched to heading: " + currentFieldHeading + "° (instruction: " + instruction + ")");
        }
    }

    /**
     * Normalizes any angle to the nearest cardinal direction (0, 90, 180, or 270 degrees).
     */
    private double normalizeToCardinalDirection(double angle) {
        // First normalize to 0-360 range
        angle = angle % 360;
        if (angle < 0) angle += 360;
        
        // Round to nearest multiple of 90
        int quadrant = (int) Math.round(angle / 90.0);
        quadrant = quadrant % 4; // Ensure 0-3 range
        
        return quadrant * 90.0;
    }

    @Override
    public void execute() {
        // Check distance and trigger direction switch if close enough
        double distance = qrSubsystem.getDistance();

        if (qrSubsystem.hasTarget()) {
            cameraScanCommand.stopScanning();
        }
        else {
            cameraScanCommand.startScanning();
        }
        
        if (distance != 0) {
            lastDistance = distance;
        }

        if ((distance > 0 && distance < SWITCH_DISTANCE_CM) || qrSubsystem.getMissedQRS() > (lastDistance - SWITCH_DISTANCE_CM)) {
            switchDirections();
        }
        
        // Drive in the current field-relative direction without rotating the robot
        Translation2d driveVector = getDriveVectorForHeading(currentFieldHeading);
        swerveDrive.processCarteseanInput(
            driveVector,
            0, // No rotation - robot orientation stays the same
            true, // Field-relative
            false // Closed loop
        );
    }

    /**
     * Calculates the drive vector for a given field heading.
     * Only supports cardinal directions: 0, 90, 180, 270 degrees.
     * 0° = forward (away from team booth)
     * 90° = right
     * 180° = backward (towards team booth)
     * 270° = left
     */
    private Translation2d getDriveVectorForHeading(double heading) {
        // Ensure heading is a cardinal direction
        heading = normalizeToCardinalDirection(heading);
        
        Rotation2d rotation = Rotation2d.fromDegrees(heading);
        // WPILib convention: Y is forward, X is right
        double xSpeed = rotation.getSin() * driveSpeed; // Right/left component
        double ySpeed = rotation.getCos() * driveSpeed; // Forward/back component
        return new Translation2d(- xSpeed, ySpeed);
    }

    @Override
    public void end(boolean interrupted) {
        swerveDrive.stop();
        swerveDrive.skipRateLimiting = false;
        cameraScanCommand.cancel();
    }

    @Override
    public boolean isFinished() {
        return "Finish".equalsIgnoreCase(lastInstruction);
    }
}
