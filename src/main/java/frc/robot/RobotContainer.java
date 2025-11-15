package frc.robot;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.CameraTowerSubsystem;
import frc.robot.subsystems.LaserTurretSubsystem;
import frc.robot.subsystems.QRDirectionSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.subsystems.network.NetworkSubsystem;
import frc.robot.subsystems.network.NetworkSubsystem.AutoMode;
import frc.robot.subsystems.network.NetworkSubsystem.TeleopMode;
import frc.robot.commands.*;
import frc.robot.commands.auto_routines.MazeAutoCommand;
import frc.robot.commands.camera.CameraScanCommand;
import frc.robot.commands.camera.CameraTurnCommand;
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

    private final Command joystickDriveCommand;
    private final Command crabDriveCommand;
    private final Command centerWheels;

    private final Command cameraTurnCommand;
    private final Command laserMoveCommand;

    private final Command mazeAutoCommand;

    private final Command zeroGyroCommand;

    public DigitalInput testLimitSwitch = new DigitalInput(0);
    private boolean lastValue = false;

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

        cameraTurnCommand = new CameraTurnCommand(cameraTower, 
            () -> controller.getLeftBumperButton() ? 0.1 : controller.getRightBumperButton() ? -0.1 : 0.0
        );
        laserMoveCommand = new LaserMoveCommand(laserTurret, 
            () -> - MathUtil.applyDeadband(controller.getRightX(), NetworkSubsystem.JOYSTICK_DEADZONE.get()),
            () -> MathUtil.applyDeadband(controller.getRightY(), NetworkSubsystem.JOYSTICK_DEADZONE.get())
        );
        joystickDriveCommand = new JoystickDriveCommand(swerve,
            () -> MathUtil.applyDeadband(controller.getLeftX(), NetworkSubsystem.JOYSTICK_DEADZONE.get()),
            () -> MathUtil.applyDeadband(controller.getLeftY(), NetworkSubsystem.JOYSTICK_DEADZONE.get()),
            () -> MathUtil.applyDeadband(controller.getLeftTriggerAxis() - controller.getRightTriggerAxis(), 
                NetworkSubsystem.TRIGGER_AXIS_DEADZONE.get())
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
        
        zeroGyroCommand = Commands.run(
        () -> {
            if (NetworkSubsystem.ZERO_ANGLE.get()) {
                swerve.zeroGyro();
                System.err.println("Swerve Drive Gyro Zeroed");
                NetworkSubsystem.ZERO_ANGLE.set(false);
            }
        });
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
            // case 
            default:
                break;
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
            case JOYSTICK_DRIVE:
                cameraTurnCommand.schedule();
                laserMoveCommand.schedule();
                joystickDriveCommand.schedule();
                break;
            case CRAB_WALK:
                crabDriveCommand.schedule();
                cameraTurnCommand.schedule();
                break;
            case CENTER_WHEELS:
                centerWheels.schedule();
                break;
            default:
                break;
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
            case MAZE:
                mazeAutoCommand.schedule();
                break;
            default:
                break;
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
