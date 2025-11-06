package frc.robot.commands;

import java.util.HashSet;
import java.util.Set;

import edu.wpi.first.wpilibj2.command.Command;

public abstract class TrackedCommand extends Command {
    public static Set<Command> trackedCommands = new HashSet<>();

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

    public static void cancelAll() {
        for (Command command : new HashSet<>(trackedCommands)) {
            command.cancel();
        }
    }
}
