package frc.robot.math;

public class MathHelper {
    public static double ScaleSpeedInput(double input) {
        double sign = Math.signum(input);
        double absInput = Math.abs(input);

        double scaledInput = Math.pow(absInput, 2.6);

        return sign * scaledInput;
    }
    public static double ScaleRotInput(double input) {
        double sign = Math.signum(input);
        double absInput = Math.abs(input);

        double scaledInput = Math.pow(absInput, 2.1);

        return sign * scaledInput;
    }
    public static double Interpolate(double min, double max, double rate) {
        if (rate < 0) return min;
        if (rate > 1) return max;

        return min + (max - min) * rate;
    }
}
