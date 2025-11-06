package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.CameraTowerSubsystem;
import frc.robot.subsystems.LaserTurretSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.subsystems.network.NetworkSubsystem;
import frc.robot.subsystems.network.NetworkSubsystem.AutoMode;
import frc.robot.commands.*;
import frc.robot.commands.drivemodes.CrabDriveCommand;
import frc.robot.commands.drivemodes.JoystickDriveCommand;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import swervelib.SwerveDriveTest;
import edu.wpi.first.math.MathUtil;

public class RobotContainer {
    private static RobotContainer instance = null;

    private final XboxController controller = new XboxController(0);

    private final SwerveDriveSubsystem swerveDriveSubsystem;
    private final LaserTurretSubsystem laserTurret = new LaserTurretSubsystem(11, 12);
    private final CameraTowerSubsystem cameraTower = new CameraTowerSubsystem(21);

    private final Command joystickDriveCommand;
    private final Command crabDriveCommand;
    private final Command centerWheels;

    private final Command cameraTurnCommand;
    private final Command laserMoveCommand;

    private final Command zeroGyroCommand;

    private AutoMode lastMode = AutoMode.NONE;
    
    public RobotContainer() {
        if (instance != null) {
            instance.close();
        }
        else {
            NetworkSubsystem.Init();
        }

        try {
            swerveDriveSubsystem = new SwerveDriveSubsystem();
        } catch (Exception e) {
            throw new RuntimeException("Failed to initialize SwerveDriveSubsystem", e);
        }

        instance = this;

        cameraTurnCommand = new CameraTurnCommand(cameraTower, 
            () -> controller.getLeftBumperButton() ? 1.0 : controller.getRightBumperButton() ? -1.0 : 0.0
        );
        laserMoveCommand = new LaserMoveCommand(laserTurret, 
            () -> - MathUtil.applyDeadband(controller.getRightX(), NetworkSubsystem.JOYSTICK_DEADZONE.get()),
            () -> MathUtil.applyDeadband(controller.getRightY(), NetworkSubsystem.JOYSTICK_DEADZONE.get())
        );
        joystickDriveCommand = new JoystickDriveCommand(swerveDriveSubsystem,
            () -> MathUtil.applyDeadband(controller.getLeftX(), NetworkSubsystem.JOYSTICK_DEADZONE.get()),
            () -> MathUtil.applyDeadband(controller.getLeftY(), NetworkSubsystem.JOYSTICK_DEADZONE.get()),
            () -> MathUtil.applyDeadband(controller.getLeftTriggerAxis() - controller.getRightTriggerAxis(), 
                NetworkSubsystem.TRIGGER_AXIS_DEADZONE.get())
        );
        crabDriveCommand = new CrabDriveCommand(swerveDriveSubsystem, 
            () -> MathUtil.applyDeadband(controller.getRightTriggerAxis(), NetworkSubsystem.TRIGGER_AXIS_DEADZONE.get()),
            () -> - controller.getPOV() * 2 * Math.PI / 360, 
            () -> - MathUtil.applyDeadband(controller.getRightX(), NetworkSubsystem.JOYSTICK_DEADZONE.get()), 
        false);
        centerWheels = Commands.run(
            () -> {
                SwerveDriveTest.centerModules(swerveDriveSubsystem.swerveDrive);
            },
        swerveDriveSubsystem);
        
        zeroGyroCommand = Commands.run(
        () -> {
            if (NetworkSubsystem.ZERO_ANGLE.get()) {
                swerveDriveSubsystem.zeroGyro();
                System.err.println("Swerve Drive Gyro Zeroed");
                NetworkSubsystem.ZERO_ANGLE.set(false);
            }
        });
    }
    public void teleopInit() {
        swerveDriveSubsystem.applyLowBatteryLimiters();

        if (!zeroGyroCommand.isScheduled()) {
            zeroGyroCommand.schedule();
        }
    }
    public void OnLastModeChange(AutoMode lastAutoMode, AutoMode newAutoMode) {
        // Cancel all active commands
        TrackedCommand.cancelAll();

        // Schedule commands based on the new mode
        switch (newAutoMode) {
            case JOYSTICK_DRIVE:
                cameraTurnCommand.schedule();
                laserMoveCommand.schedule();
                joystickDriveCommand.schedule();
                break;
            case CRAB_WALK:
                crabDriveCommand.schedule();
                cameraTurnCommand.schedule();
            case CENTER_WHEELS:
                centerWheels.schedule();
                break;
            default:
                break;
        }
    }
    public void processManualInput() {
        AutoMode currentMode = NetworkSubsystem.autoModeChooser.getSelected();

        if (lastMode != currentMode) {
            OnLastModeChange(lastMode, currentMode);
            lastMode = NetworkSubsystem.autoModeChooser.getSelected();
        }
    }

    public void close() {
    }
    public static RobotContainer getInstance() {
        if (instance == null) {
            instance = new RobotContainer();
        }
        return instance;
    }
}
