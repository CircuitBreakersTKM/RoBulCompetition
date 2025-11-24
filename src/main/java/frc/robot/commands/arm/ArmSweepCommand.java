package frc.robot.commands.arm;

import java.util.function.DoubleSupplier;

import frc.robot.commands.TrackedCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.network.NetworkSubsystem;

/**
 * Command that snaps the arm to specific angles using proportional speed control.
 * Takes the shortest rotational path and smoothly decelerates as it approaches the target.
 */
public class ArmSweepCommand extends TrackedCommand { 
    private final ArmSubsystem armSubsystem;
    private final DoubleSupplier armSpeedSupplier;
    private final DoubleSupplier sweepSpeedSupplier;
    
    /**
     * Creates a new ArmSnapPickupCommand.
     * 
     * @param armSubsystem The arm subsystem
     * @param pickupSupplier Supplier that returns true if holding an object, false otherwise
     * @param positionSupplier Supplier that provides the desired arm position (DOWN, HANGING, or RAISED)
     */
    public ArmSweepCommand(ArmSubsystem armSubsystem, DoubleSupplier armSpeedSupplier, DoubleSupplier sweepSpeedSupplier) {
        this.armSubsystem = armSubsystem;
        this.armSpeedSupplier = armSpeedSupplier;
        this.sweepSpeedSupplier = sweepSpeedSupplier;
        
        addRequirements(armSubsystem);
    }
    
    @Override
    public void initialize() {
    }
    
    @Override
    public void execute() {
        double sweepSpeed = NetworkSubsystem.ARM_BRUSH_SPEED.get();
        double swingSpeed = NetworkSubsystem.ARM_SWING_SPEED.get();

        armSubsystem.setSpeed(armSpeedSupplier.getAsDouble() * swingSpeed, sweepSpeedSupplier.getAsDouble() * sweepSpeed);
    }
    
    @Override
    public void end(boolean interrupted) {
        armSubsystem.stop();
    }
    
    @Override
    public boolean isFinished() {
        return false; // Runs until cancelled
    }
}
