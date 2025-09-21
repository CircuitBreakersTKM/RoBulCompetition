package frc.robot;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.XboxController;
import swervelib.SwerveDrive;

public class RobotContainer {
    private static RobotContainer instance = null;

    private final XboxController controller = new XboxController(0);
    private final double maxSpeed = 0.4;
    private final double maxRot = 0.4;
    private final double deadZone = 0.25;
    
    //private final SwerveDrive swerve;
    private SparkMax[] motors = new SparkMax[8];
    
    public RobotContainer() {
        if (instance != null) {
            instance.close();
        }
        instance = this;

        for (int i = 0; i < 8; i++) {
            motors[i] = new SparkMax(i + 1, SparkMax.MotorType.kBrushless);
        }
    }
    public void processManualInput() {
        double LeftX = controller.getLeftX();
        double LeftY = controller.getLeftY();

        if (Math.abs(LeftY) < deadZone) {
            LeftY = 0;
        }
        if (Math.abs(LeftX) < deadZone) {
            LeftX = 0;
        }

        for (int i = 0; i < 4; i++) {
            motors[i].set(-LeftY * maxSpeed); // Forward/backward => Forward is negative
        }
        for (int i = 4; i < 8; i++) {
            motors[i].set(-LeftX * maxRot); // Left/right => Right is positive but wheels reverse the movement
        }
    }

    public void close() {
        for (int i = 0; i < 8; i++) {
            motors[i].set(0);
            motors[i].close();
        }
    }
    public static RobotContainer getInstance() {
        if (instance == null) {
            instance = new RobotContainer();
        }
        return instance;
    }
}
