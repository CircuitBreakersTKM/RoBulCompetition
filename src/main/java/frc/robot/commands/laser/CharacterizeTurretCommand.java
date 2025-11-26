package frc.robot.commands.laser;

import frc.robot.commands.TrackedCommand;
import frc.robot.subsystems.LaserTurretSubsystem;

public class CharacterizeTurretCommand extends TrackedCommand {

    private final LaserTurretSubsystem turret;
    private final boolean azimuth; // true = az, false = alt

    private double voltage = 0.0;
    private boolean foundKs = false;

    private int counter = 0;

    private boolean done = false;

    public CharacterizeTurretCommand(LaserTurretSubsystem turret, boolean azimuth) {
        this.turret = turret;
        this.azimuth = azimuth;
        addRequirements(turret);
    }

    @Override
    public void initialize() {
        System.out.println("\n==== CHARACTERIZATION START ====");
        System.out.println("Motor: " + (azimuth ? "Azimuth" : "Altitude"));
        System.out.println("Sweeping voltage from 0 → 2V to detect kS...\n");
    }

    @Override
    public void execute() {
        double pos;
        double vel;

        if (azimuth) {
            pos = turret.azEncoder.getPosition();
            vel = turret.azEncoder.getVelocity() / 60.0; // revs/sec
        } else {
            pos = turret.altEncoder.getPosition();
            vel = turret.altEncoder.getVelocity() / 60.0;
        }

        if (counter < 20) {
            counter++;
            return;
        }

        // STEP 1: Sweep voltage until turret starts moving → kS
        if (!foundKs) {
            counter++;

            if (counter % 5 == 1) {
                voltage += 0.001;
            }

            if (voltage > 2.0) voltage = 2.0;

            if (azimuth) {
                turret.azimuthMotor.setVoltage(voltage);
            } else {
                turret.altitudeMotor.setVoltage(voltage);
            }

            // turret moves when velocity escapes noise floor
            if (Math.abs(vel) > 0.05) {
                System.out.println("kS detected at ~ " + voltage + " volts");

                counter = 0;
                foundKs = true;

                System.out.println("\nNow holding at 1.0V to measure kV...");
                voltage = 1.0;
            }

            return;
        }

        // STEP 2: Hold constant voltage to measure kV
        if (azimuth) {
            turret.azimuthMotor.setVoltage(voltage);
        } else {
            turret.altitudeMotor.setVoltage(voltage);
        }

        if (counter < 40) {
            counter++;
        }
        else {
            done = true;
        }

        // print velocity every loop
        System.out.println(String.format("Voltage: %.2f | Velocity: %.4f rps", voltage, vel));

    }

    @Override
    public void end(boolean interrupted) {
        turret.stop();
        System.out.println("\n==== CHARACTERIZATION END ====\n");
    }

    @Override
    public boolean isFinished() {
        return done;
    }
}
