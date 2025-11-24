package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.interfaces.MotorizedSubsystem;

public class ArmSubsystem extends SubsystemBase implements MotorizedSubsystem {
    public SparkMax armSwingMotor;
    public SparkMax sweepMotor;
    public DigitalInput armSwingLimitSwitch; // this button is normally closed, so when pressed it will return false
    
    private static final double SWING_GEAR_RATIO = 64.0; // 64 motor rotations = 1 arm rotation
    
    private final RelativeEncoder swingEncoder;
    private final SparkClosedLoopController swingPIDController;
    
    // Position PID constants for holding (slot 0)
    private static final double kP_POS = 0.12;
    private static final double kI_POS = 0.0;
    private static final double kD_POS = 0.01;
    
    // Gravity compensation
    private static final double kG = 0.05;
    private static final double gravityCompensationSpeed = 0.15;

    private boolean holdingPosition = false;
    private double holdPosition = 0.0;

    public ArmSubsystem(int armSwingMotorPort, int sweepMotorPort, int armSwingLimitSwitchDIOPort) {
        armSwingMotor = new SparkMax(armSwingMotorPort, MotorType.kBrushless);
        sweepMotor = new SparkMax(sweepMotorPort, MotorType.kBrushless);
        armSwingLimitSwitch = new DigitalInput(armSwingLimitSwitchDIOPort);
        
        swingEncoder = armSwingMotor.getEncoder();
        swingPIDController = armSwingMotor.getClosedLoopController();
        
        // Configure swing motor with dual PID slots
        SparkMaxConfig swingConfig = new SparkMaxConfig();
        swingConfig.inverted(true);
        
        // Slot 0: Position control for movement
        swingConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(kP_POS, kI_POS, kD_POS);

        armSwingMotor.configure(swingConfig, SparkMax.ResetMode.kResetSafeParameters, SparkMax.PersistMode.kPersistParameters);
        
        // Configure sweep motor
        SparkMaxConfig sweepConfig = new SparkMaxConfig();
        sweepConfig.inverted(true);
        sweepMotor.configure(sweepConfig, SparkMax.ResetMode.kResetSafeParameters, SparkMax.PersistMode.kPersistParameters);
        
        armSwingMotor.getEncoder().setPosition(0);
        sweepMotor.getEncoder().setPosition(0);
    }

    @Override
    public void periodic() {
        if (!armSwingLimitSwitch.get()) { // remember, the limit switch is normally closed
            armSwingMotor.getEncoder().setPosition(0);
        }
    }

    public void stopSweepMotor() {
        sweepMotor.stopMotor();
    }

    @Override
    public void setSpeed(double... speeds) {
        if (speeds.length != 2) {
            throw new IllegalArgumentException("Expected 2 speed values for ArmSubsystem");
        }

        double swingSpeed = speeds[0];
        
        if (!armSwingLimitSwitch.get() && swingSpeed < 0) {
            holdingPosition = false;

            // Prevent moving arm down if limit switch is pressed - hold position
            armSwingMotor.set(0);
        } else if (Math.abs(swingSpeed) < 0.03) {

            // if (!holdingPosition) {
            //     // Just stopped moving - capture hold position
            //     holdPosition = swingEncoder.getPosition();
            //     holdingPosition = true;
            // }

            // // Speed is ~0: Use position control to hold current position
            // double angleDeg = getSwingEncoderPosition();
            // double gravityFF = kG * Math.sin(Math.toRadians(angleDeg + 90));

            // swingPIDController.setReference(holdPosition, ControlType.kPosition, ClosedLoopSlot.kSlot0, gravityFF, SparkClosedLoopController.ArbFFUnits.kPercentOut);
            
            armSwingMotor.set(0);
        } else {
            holdingPosition = false;
            
            armSwingMotor.set(swingSpeed);
        }
        
        sweepMotor.set(speeds[1]);
    }

    @Override
    public void stop() {
        holdingPosition = false;
        armSwingMotor.stopMotor(); 
        sweepMotor.stopMotor();
    }

    @Override
    public void stopIf(boolean condition) {
        if (condition) {
            stop();
        }
    }
    
    /**
     * Gets the current arm swing encoder position in degrees.
     * 
     * @return The arm position in degrees (0-360 range based on encoder rotations)
     */
    public double getSwingEncoderPosition() {
        return swingEncoder.getPosition() / SWING_GEAR_RATIO * 360;
    }
}
