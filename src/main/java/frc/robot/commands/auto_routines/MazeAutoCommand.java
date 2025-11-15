package frc.robot.commands.auto_routines;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.commands.TrackedCommand;
import frc.robot.subsystems.CameraTowerSubsystem;
import frc.robot.subsystems.QRDirectionSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.subsystems.network.NetworkSubsystem;
import frc.robot.commands.camera.CameraScanCommand;

public class MazeAutoCommand extends TrackedCommand {
    private final SwerveDriveSubsystem swerveDrive;
    private final CameraScanCommand cameraScanCommand;
    private final QRDirectionSubsystem qrSubsystem;
    
    private double driveSpeed;
    private double currentFieldHeading;

    private String lastInstruction;
    private long lastQRTimestamp = 0;
    private double lastDistance = Double.MAX_VALUE;
    private int lastTicker = 0;

    private static final double SWITCH_DISTANCE_CM = 62.5;

    public MazeAutoCommand(
            SwerveDriveSubsystem swerveDrive, 
            CameraTowerSubsystem cameraTower,
            QRDirectionSubsystem qrSubsystem,
            double driveSpeed) {
        this.swerveDrive = swerveDrive;
        this.cameraScanCommand = new CameraScanCommand(cameraTower, () -> this.currentFieldHeading, 12);
        this.qrSubsystem = qrSubsystem;
        this.driveSpeed = driveSpeed;
        this.currentFieldHeading = 0;
        
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
        lastDistance = Double.MAX_VALUE;
        lastQRTimestamp = 0;
        lastTicker = 0;
        
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
            lastDistance = distance;
            lastQRTimestamp = System.currentTimeMillis();
            lastTicker = qrSubsystem.getTicker();
        }

        if (qrSubsystem.hasTarget()) {
            cameraScanCommand.stopScanning();
        } else {
            cameraScanCommand.startScanning();
        }

        if (lastDistance < SWITCH_DISTANCE_CM - (System.currentTimeMillis() - lastQRTimestamp) * driveSpeed * 100 / 1000) {
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
