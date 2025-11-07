package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.CameraTowerSubsystem;
import frc.robot.subsystems.LaserTurretSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.subsystems.network.NetworkSubsystem;
import frc.robot.subsystems.network.NetworkSubsystem.AutoMode;
import frc.robot.commands.*;
import frc.robot.commands.drive_modes.CrabDriveCommand;
import frc.robot.commands.drive_modes.JoystickDriveCommand;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import swervelib.SwerveDriveTest;

import edu.wpi.first.math.MathUtil;

public class RobotContainer {
    private static RobotContainer instance = null;

    private final XboxController controller = new XboxController(0);

    private final SwerveDriveSubsystem swerve;
    private final LaserTurretSubsystem laserTurret = new LaserTurretSubsystem(11, 12);
    private final CameraTowerSubsystem cameraTower = new CameraTowerSubsystem(21);

    private final Command joystickDriveCommand;
    private final Command crabDriveCommand;
    private final Command centerWheels;

    private final Command cameraTurnCommand;
    private final Command cameraSnapCommand;
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
            swerve = new SwerveDriveSubsystem();
        } catch (Exception e) {
            throw new RuntimeException("Failed to initialize SwerveDriveSubsystem", e);
        }

        instance = this;

        cameraTurnCommand = new CameraTurnCommand(cameraTower, 
            () -> controller.getLeftBumperButton() ? 1.0 : controller.getRightBumperButton() ? -1.0 : 0.0
        );
        cameraSnapCommand = new CameraSnapCommand(cameraTower, 
            () -> - controller.getPOV()
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
        
        zeroGyroCommand = Commands.run(
        () -> {
            if (NetworkSubsystem.ZERO_ANGLE.get()) {
                swerve.zeroGyro();
                System.err.println("Swerve Drive Gyro Zeroed");
                NetworkSubsystem.ZERO_ANGLE.set(false);
            }
        });
    }
    public void teleopInit() {
        swerve.applyLowBatteryLimiters();
        cameraTower.zeroEncoder();

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
                break;
            case CENTER_WHEELS:
                centerWheels.schedule();
                break;
            case CAMERA_TOWER_TEST:
                cameraSnapCommand.schedule();
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

        // switch (currentMode) {
        //     case CAMERA_TOWER_TEST -> {
        //         double currentPosition = cameraTower.azimuthMotor.getEncoder().getPosition() / 100 * 360;
        //         double povInput = -controller.getPOV();
        //         double allowedError = 2; //deg
        //         double maxSpeed = 1;
        //         double minSpeed = 0.05; // Minimum speed to prevent stalling
        //         double slowdownRange = 25; // Start slowing down 30 degrees before target

        //         // If POV pressed, update target angle
        //         if (povInput != 1) {
        //             // Normalize desired position to 0-360 range
        //             cameraTargetAngle = povInput % 360;
        //             if (cameraTargetAngle < 0) cameraTargetAngle += 360;
        //         }

        //         // Scale positions so current is always at 180° to find shortest path
        //         double scaledCurrent = 180;
        //         double scaledDesired = cameraTargetAngle - currentPosition + 180;
                
        //         // Normalize scaled desired to 0-360 range
        //         scaledDesired = scaledDesired % 360;
        //         if (scaledDesired < 0) scaledDesired += 360;

        //         // Calculate error (shortest path)
        //         double error = scaledDesired - scaledCurrent;

        //         if (Math.abs(error) < allowedError) {
        //             cameraTower.setSpeed(0);
        //             break;
        //         }
                
        //         // Scale speed proportionally to distance
        //         double speed;
        //         if (Math.abs(error) > slowdownRange) {
        //             // Full speed when far from target
        //             speed = maxSpeed;
        //         } else {
        //             // Proportional speed when close to target
        //             speed = minSpeed + (maxSpeed - minSpeed) * (Math.abs(error) / slowdownRange);
        //         }
                
        //         cameraTower.setSpeed(Math.copySign(speed, error));
        //     }
        //     default -> {}
        // }
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
