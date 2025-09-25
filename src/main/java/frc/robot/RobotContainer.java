package frc.robot;

import static edu.wpi.first.units.Units.Rotation;

import java.io.File;
import edu.wpi.first.wpilibj.XboxController;
import swervelib.SwerveDrive;
import swervelib.SwerveModule;
import edu.wpi.first.wpilibj.Filesystem;
import swervelib.parser.SwerveParser;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class RobotContainer {
    private static RobotContainer instance = null;

    private final XboxController controller = new XboxController(0);
    private SwerveDrive swerveDrive;

    private SwerveModule[] swerveModules;
    
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

        swerveModules = swerveDrive.getModules();

    }
    public void processManualInput() {
        /*double LeftX = controller.getLeftX();
        double LeftY = controller.getLeftY();

        double rot = controller.getRightX();

        swerveDrive.drive(new Translation2d(LeftX, -LeftY), rot, false, true);
         */

        //swerveModules[0].setAngle(0);     // Nope
        //swerveModules[0].setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(0)), trie, 0);   // Nope
        // Increasing speed = nope

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
