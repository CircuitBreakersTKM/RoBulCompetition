package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LaserTurretSubsystem extends SubsystemBase {

    private static final double AZIMUTH_GEAR_RATIO = 100.0;
    private static final double ALTITUDE_GEAR_RATIO = 100.0;

    // Azimuth (horizontal)
    private static final double AZ_kP = 0.18;
    private static final double AZ_kI = 0.0;
    private static final double AZ_kD = 0.01;
    private static final double AZ_kF = 0.0;

    // Altitude (vertical)
    private static final double ALT_kP = 0.18;
    private static final double ALT_kI = 0.0;
    private static final double ALT_kD = 0.01;
    private static final double ALT_kF = 0.0;

    public final SparkMax azimuthMotor;
    public final SparkMax altitudeMotor;

    private final RelativeEncoder azEncoder;
    private final RelativeEncoder altEncoder;

    public LaserTurretSubsystem(int azimuthCanID, int altitudeCanID) {

        azimuthMotor  = new SparkMax(azimuthCanID, MotorType.kBrushless);
        altitudeMotor = new SparkMax(altitudeCanID, MotorType.kBrushless);

        azEncoder  = azimuthMotor.getEncoder();
        altEncoder = altitudeMotor.getEncoder();

        // Configure azimuth motor
        SparkMaxConfig azConfig = new SparkMaxConfig();
        azConfig
            .inverted(true)
            .closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pidf(AZ_kP, AZ_kI, AZ_kD, AZ_kF);
        
        azimuthMotor.configure(azConfig, SparkMax.ResetMode.kResetSafeParameters, SparkMax.PersistMode.kPersistParameters);

        // Configure altitude motor
        SparkMaxConfig altConfig = new SparkMaxConfig();
        altConfig
            .inverted(true)
            .closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pidf(ALT_kP, ALT_kI, ALT_kD, ALT_kF);
        
        altitudeMotor.configure(altConfig, SparkMax.ResetMode.kResetSafeParameters, SparkMax.PersistMode.kPersistParameters);
    }

    // ---------- ANGLE HELPERS ----------

    public double getAzimuthDegrees() {
        return azEncoder.getPosition() * 360.0 / AZIMUTH_GEAR_RATIO;
    }

    public double getAltitudeDegrees() {
        return altEncoder.getPosition() * 360.0 / ALTITUDE_GEAR_RATIO;
    }

    public void setAzimuthTarget(double deg) {
        double motorRevs = deg / 360.0 * AZIMUTH_GEAR_RATIO;
        azimuthMotor.getClosedLoopController().setReference(motorRevs, SparkBase.ControlType.kPosition, ClosedLoopSlot.kSlot0);
    }

    public void setAltitudeTarget(double deg) {
        double motorRevs = deg / 360.0 * ALTITUDE_GEAR_RATIO;
        altitudeMotor.getClosedLoopController().setReference(motorRevs, SparkBase.ControlType.kPosition, ClosedLoopSlot.kSlot0);
    }

    public void setSpeed(double azimuthSpeed, double altitudeSpeed) {
        azimuthMotor.set(azimuthSpeed);
        altitudeMotor.set(altitudeSpeed);
    }

    public void stop() {
        azimuthMotor.stopMotor();
        altitudeMotor.stopMotor();
    }
}
