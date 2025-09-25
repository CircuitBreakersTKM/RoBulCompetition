package frc.robot;

import java.io.File;
import edu.wpi.first.wpilibj.XboxController;
import swervelib.SwerveDrive;
import edu.wpi.first.wpilibj.Filesystem;
import swervelib.parser.SwerveParser;
import edu.wpi.first.math.geometry.Translation2d;

public class RobotContainer {
    private static RobotContainer instance = null;

    private final XboxController controller = new XboxController(0);
    private SwerveDrive swerveDrive;
    
    public RobotContainer() {
        if (instance != null) {
            instance.close();
        }
        instance = this;

        // Specify the directory containing your JSON configuration files
        File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(), "swerve");

        // Initialize the swerve drive using the configuration files
        swerveDrive = null;

        try{
            swerveDrive = new SwerveParser(swerveJsonDirectory).createSwerveDrive(10);
        } catch (Exception e) {
            System.out.println("Error initializing SwerveDrive: " + e.getMessage());
        }

    }
    public void processManualInput() {
        double LeftX = controller.getLeftX();
        double LeftY = controller.getLeftY();

        double rot = controller.getRightX();

        swerveDrive.drive(new Translation2d(LeftX, -LeftY), rot, false, true);
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
