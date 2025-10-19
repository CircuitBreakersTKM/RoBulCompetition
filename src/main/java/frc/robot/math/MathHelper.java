package frc.robot.math;

public class MathHelper {
    public static double ScaleInput(double input) {
        double sign = Math.signum(input);
        double absInput = Math.abs(input);

        double scaledInput = Math.pow(absInput, 2.4);

        return sign * scaledInput;
    }
}
