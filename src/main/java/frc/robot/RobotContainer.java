package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.CameraTowerSubsystem;
import frc.robot.subsystems.LaserTurretSubsystem;
import frc.robot.subsystems.QRDirectionSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.subsystems.network.NetworkSubsystem;
import frc.robot.subsystems.network.NetworkSubsystem.AutoMode;
import frc.robot.subsystems.network.NetworkSubsystem.TeleopMode;
import frc.robot.commands.*;
import frc.robot.commands.arm.ArmSweepCommand;
import frc.robot.commands.auto_routines.MazeAutoCommand;
import frc.robot.commands.auto_routines.PointAtBlobCommand;
import frc.robot.commands.drive_modes.CrabDriveCommand;
import frc.robot.commands.drive_modes.JoystickDriveCommand;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import swervelib.SwerveDriveTest;

import edu.wpi.first.math.MathUtil;

/**
 * Container class that holds all subsystems, commands, and controller mappings.
 * Implements singleton pattern to ensure only one instance exists.
 * Manages auto mode switching and controller input processing.
 */
public class RobotContainer {
    private static RobotContainer instance = null;

    private final XboxController controller = new XboxController(0);

    private final SwerveDriveSubsystem swerve;
    private final LaserTurretSubsystem laserTurret = new LaserTurretSubsystem(11, 12);
    private final CameraTowerSubsystem cameraTower = new CameraTowerSubsystem(21);
    private final QRDirectionSubsystem qrDirectionSubsystem = new QRDirectionSubsystem();
    private final ArmSubsystem armSubsystem = new ArmSubsystem(31, 32, 0);

    private final Command joystickDriveCommand;
    private Command crabDriveCommand;
    private final Command centerWheels;

    private Command laserMoveCommand;

    private final Command mazeAutoCommand;
    private final Command lookAtBlobCommand;
    
    private final Command armSnapPickupCommand;

    private final Command zeroGyroCommand;

    private TeleopMode lastTeleopMode = TeleopMode.NONE;
    private AutoMode lastAutoMode = AutoMode.NONE;
    
    /**
     * Private constructor initializes all subsystems and commands.
     * Sets up controller bindings and command suppliers with deadband applied.
     */
    public RobotContainer() {
        if (instance != null) {
            instance.close();
        }
        else {
            NetworkSubsystem.Init();
        }

        try {
            swerve = new SwerveDriveSubsystem();
        } catch (Exception e) {
            throw new RuntimeException("Failed to initialize SwerveDriveSubsystem", e);
        }

        instance = this;

        laserMoveCommand = new LaserMoveCommand(laserTurret, 
            () -> - MathUtil.applyDeadband(controller.getRightX(), NetworkSubsystem.JOYSTICK_DEADZONE.get()),
            () -> MathUtil.applyDeadband(controller.getLeftY(), NetworkSubsystem.JOYSTICK_DEADZONE.get())
        );
        joystickDriveCommand = new JoystickDriveCommand(swerve,
            () -> MathUtil.applyDeadband(controller.getLeftX(), NetworkSubsystem.JOYSTICK_DEADZONE.get()),
            () -> MathUtil.applyDeadband(controller.getLeftY(), NetworkSubsystem.JOYSTICK_DEADZONE.get()),
            () -> MathUtil.applyDeadband(controller.getLeftTriggerAxis() - controller.getRightTriggerAxis(), 
                NetworkSubsystem.TRIGGER_AXIS_DEADZONE.get()),
            () -> !controller.getLeftBumperButton()
        );
        crabDriveCommand = new CrabDriveCommand(swerve, 
            () -> MathUtil.applyDeadband(controller.getRightTriggerAxis(), NetworkSubsystem.TRIGGER_AXIS_DEADZONE.get()),
            () -> - controller.getPOV(), 
            () -> - MathUtil.applyDeadband(controller.getRightX(), NetworkSubsystem.JOYSTICK_DEADZONE.get()), 
        false);
        centerWheels = Commands.run(
            () -> {
                SwerveDriveTest.centerModules(swerve.swerveDrive);
            },
        swerve);

        mazeAutoCommand = new MazeAutoCommand(
            swerve,
            cameraTower,
            qrDirectionSubsystem,
            0.5
        );
        lookAtBlobCommand = new PointAtBlobCommand(laserTurret);
        
        double pickUpSpeed = 1.0;
        double dropOffSpeed = 0.6;
        armSnapPickupCommand = new ArmSweepCommand(
            armSubsystem,
            () -> num(controller.getXButton()) - num(controller.getBButton()),
            () -> num(controller.getAButton()) * pickUpSpeed - num(controller.getYButton()) * dropOffSpeed
        );
        
        zeroGyroCommand = Commands.run(
        () -> {
            if (NetworkSubsystem.ZERO_ANGLE.get()) {
                swerve.zeroGyro();
                System.err.println("Swerve Drive Gyro Zeroed");
                NetworkSubsystem.ZERO_ANGLE.set(false);
            }
        });
    }

    private double num(boolean b) {
        return b ? 1.0 : 0.0;
    }
    
    public void init() {
        if (!NetworkSubsystem.OVERRIDE_LOW_VOLTAGE_LIMIERS.get()) {
            swerve.applyLowBatteryLimiters();
        }

        if (!zeroGyroCommand.isScheduled()) {
            zeroGyroCommand.schedule();
        }
    }
    
    /**
     * Called periodically during teleop.
     * Monitors auto mode selection and triggers mode change handler when mode changes.
     */
    public void teleopPeriodic() {
        TeleopMode currentMode = NetworkSubsystem.teleopModeChooser.getSelected();

        if (lastTeleopMode != currentMode) {
            OnLastModeChange(lastTeleopMode, currentMode);
            lastTeleopMode = NetworkSubsystem.teleopModeChooser.getSelected();
        }

        switch (currentMode) {
            case KROUZKY -> {
                
            }
            default -> {}
        }
    }

    /**
     * Handles auto mode transitions by canceling all active commands
     * and scheduling commands appropriate for the new mode.
     * 
     * @param lastMode The previous auto mode
     * @param newMode The new auto mode to activate
     */
    public void OnLastModeChange(TeleopMode lastMode, TeleopMode newMode) {
        // Cancel all active commands
        TrackedCommand.cancelAll();

        // Schedule commands based on the new mode
        switch (newMode) {
            case LASERY_POLE -> {
                crabDriveCommand = new CrabDriveCommand(swerve, 
                    () -> MathUtil.applyDeadband(controller.getRightTriggerAxis(), NetworkSubsystem.TRIGGER_AXIS_DEADZONE.get()),
                    () -> - controller.getPOV(), 
                    () -> - MathUtil.applyDeadband(controller.getRightX(), NetworkSubsystem.JOYSTICK_DEADZONE.get()), 
                    false);

                crabDriveCommand.schedule();
            }
            case LASERY_VEZ -> {
                laserMoveCommand = new LaserMoveCommand(laserTurret, 
                    () -> MathUtil.applyDeadband(controller.getRightX(), NetworkSubsystem.JOYSTICK_DEADZONE.get()),
                    () -> - MathUtil.applyDeadband(controller.getLeftY(), NetworkSubsystem.JOYSTICK_DEADZONE.get())
                );

                laserMoveCommand.schedule();
            }
            case KROUZKY -> {
                armSnapPickupCommand.schedule();
                joystickDriveCommand.schedule();
            }
            case BLUDISTE -> {
                crabDriveCommand.schedule();
            }
            case PREDMETY -> {
                crabDriveCommand.schedule();
            }
            case SLALOM -> {
                laserMoveCommand = new LaserMoveCommand(laserTurret, 
                    () -> - MathUtil.applyDeadband(controller.getRightX(), NetworkSubsystem.JOYSTICK_DEADZONE.get()),
                    () -> MathUtil.applyDeadband(controller.getRightY(), NetworkSubsystem.JOYSTICK_DEADZONE.get())
                );
                crabDriveCommand = new CrabDriveCommand(swerve, 
                    () -> MathUtil.applyDeadband(controller.getRightTriggerAxis(), NetworkSubsystem.TRIGGER_AXIS_DEADZONE.get()),
                    () -> - controller.getPOV(), 
                    () -> - MathUtil.applyDeadband(controller.getLeftX(), NetworkSubsystem.JOYSTICK_DEADZONE.get()), 
                    false);

                crabDriveCommand.schedule();
            }
            case CENTER_WHEELS -> {
                centerWheels.schedule();
            }
            case TEST -> {
                armSnapPickupCommand.schedule();
            }
            default -> {}
        }
    }

    public void autonomousPeriodic() {
        AutoMode currentMode = NetworkSubsystem.autoModeChooser.getSelected();

        if (lastAutoMode != currentMode) {
            OnLastModeChange(lastAutoMode, currentMode);
            lastAutoMode = NetworkSubsystem.autoModeChooser.getSelected();
        }
    }
    public void OnLastModeChange(AutoMode lastMode, AutoMode newMode) {
        // Cancel all active commands
        TrackedCommand.cancelAll();

        // Schedule commands based on the new mode
        switch (newMode) {
            case MAZE -> {
                mazeAutoCommand.schedule();
            }
            case TEST -> {
                lookAtBlobCommand.schedule();
            }
            default -> {}
        }
    }

    /**
     * Cleanup method called when replacing a RobotContainer instance.
     */
    public void close() {
    }
    
    /**
     * Gets the singleton instance of RobotContainer, creating it if it doesn't exist.
     * 
     * @return The RobotContainer singleton instance
     */
    public static RobotContainer getInstance() {
        if (instance == null) {
            instance = new RobotContainer();
        }
        return instance;
    }
}
