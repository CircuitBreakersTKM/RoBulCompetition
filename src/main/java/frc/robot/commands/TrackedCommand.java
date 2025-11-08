package frc.robot.commands;

import java.util.HashSet;
import java.util.Set;

import edu.wpi.first.wpilibj2.command.Command;

/**
 * Abstract base class for commands that need to be tracked for bulk cancellation.
 * Maintains a static set of all scheduled TrackedCommand instances, allowing
 * cancellation of all tracked commands at once during mode transitions.
 */
public abstract class TrackedCommand extends Command {
    public static Set<Command> trackedCommands = new HashSet<>();

    /**
     * Schedules this command and adds it to the tracked commands set.
     */
    @Override
    public void schedule() {
        trackedCommands.add(this);
        super.schedule();
    }

    @Override
    public void end(boolean interrupted) {
        trackedCommands.remove(this);
        super.end(interrupted);
    }

    @Override
    public void cancel() {
        trackedCommands.remove(this);
        super.cancel();
    }

    /**
     * Cancels all currently tracked commands.
     * Useful for mode transitions where all previous commands should be stopped.
     */
    public static void cancelAll() {
        for (Command command : new HashSet<>(trackedCommands)) {
            command.cancel();
        }
    }
}
