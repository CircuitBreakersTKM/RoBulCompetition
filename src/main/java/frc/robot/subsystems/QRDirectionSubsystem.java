package frc.robot.subsystems;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Subsystem that continuously monitors QR codes from the coprocessor
 * and tracks the most frequently seen direction instructions.
 * 
 * The coprocessor publishes QR data to NetworkTables with:
 * - "Camera Tower/QR Detector/hasTarget" (boolean): whether a QR is visible
 * - "Camera Tower/QR Detector/data" (string): the QR code content
 * - "Camera Tower/QR Detector/ticker" (number): increments with each new detection
 * - "Camera Tower/QR Detector/distance" (number): estimated distance to QR code in centimeters
 */
public class QRDirectionSubsystem extends SubsystemBase {
    // Robot geometry constants (all in centimeters)
    private static final double ROBOT_LENGTH_CM = 63.5;  // Front to back
    private static final double ROBOT_WIDTH_CM = 63.5;   // Left to right
    private static final double CAMERA_OFFSET_X_CM = -0.25; // Camera offset from center (positive = right)
    private static final double CAMERA_OFFSET_Y_CM = 3.75; // Camera offset from center (positive = forward)
    
    private final NetworkTable qrTable;
    private final String prefix = "Camera Tower/QR Detector/";

    private boolean counted = false;
    private int missed = 0;
    
    private final Map<String, Integer> directionCounts;
    private double lastTicker;
    
    public QRDirectionSubsystem() {
        qrTable = NetworkTableInstance.getDefault().getTable("CircuitBreakers");
        directionCounts = new HashMap<>();
        lastTicker = -1;
        
        // Initialize direction counts
        resetCounts();
    }
    
    @Override
    public void periodic() {        
        if (hasTarget()) {
            counted = true;
            missed = 0;
            double currentTicker = qrTable.getEntry(prefix + "ticker").getDouble(-1);
            
            // Only process if ticker has changed (new detection)
            if (currentTicker != lastTicker) {
                String qrData = qrTable.getEntry(prefix + "data").getString("");
                
                // Validate the QR data is a valid direction
                if (isValidDirection(qrData)) {
                    // Increment count for this direction
                    directionCounts.put(qrData, directionCounts.getOrDefault(qrData, 0) + 1);
                }
                
                lastTicker = currentTicker;
            }
        }
        else if (counted) {
            missed++;
        }
    }
    
    /**
     * Gets the most frequently detected QR direction and resets the counts.
     * 
     * @return The direction string ("Left", "Right", or "Finish") with the highest count,
     *         or an empty string if no valid directions have been detected
     */
    public String getQRDirection() {
        String mostCommonDirection = "";
        int maxCount = 0;
        
        // Find direction with highest count
        for (Map.Entry<String, Integer> entry : directionCounts.entrySet()) {
            if (entry.getValue() > maxCount) {
                maxCount = entry.getValue();
                mostCommonDirection = entry.getKey();
            }
        }
        
        // Reset counts and ticker for next reading
        resetCounts();
        resetCoprocessorTicker();
        missed = 0;
        counted = false;
        
        System.out.println("[QR] Returning direction: " + mostCommonDirection + " (count was: " + maxCount + ")");
        return mostCommonDirection;
    }

    public boolean hasTarget() {
        return qrTable.getEntry(prefix + "hasTarget").getBoolean(false);
    }
    
    /**
     * Manually resets the internal direction counts without returning anything.
     */
    public void resetCounts() {
        directionCounts.clear();
        directionCounts.put("Left", 0);
        directionCounts.put("Right", 0);
        directionCounts.put("Finish", 0);
        lastTicker = -1;
    }
    
    /**
     * Signals the coprocessor to reset its ticker.
     * The coprocessor will see this flag and reset its ticker to 0.
     */
    private void resetCoprocessorTicker() {
        qrTable.getEntry(prefix + "resetTicker").setBoolean(true);
    }

    public int getMissedQRS() {
        return missed;
    }
    
    /**
     * Checks if the given string is a valid direction command.
     */
    private boolean isValidDirection(String data) {
        if (data == null || data.isEmpty()) {
            return false;
        }
        return data.equalsIgnoreCase("Left") || 
               data.equalsIgnoreCase("Right") || 
               data.equalsIgnoreCase("Finish");
    }
    
    /**
     * Gets the current count for a specific direction without resetting.
     * Useful for debugging or displaying on dashboard.
     */
    public int getDirectionCount(String direction) {
        return directionCounts.getOrDefault(direction, 0);
    }
    
    /**
     * Gets the total number of valid QR codes detected since last reset.
     */
    public int getTotalDetections() {
        return directionCounts.values().stream().mapToInt(Integer::intValue).sum();
    }
    
    /**
     * Checks if any QR codes have been detected since the last reset.
     */
    public boolean hasDetections() {
        return getTotalDetections() > 0;
    }
    
    /**
     * Gets the current estimated distance from the camera to the QR code in centimeters.
     * Returns 0.0 if no QR code is currently detected.
     */
    public double getDistance() {
        return qrTable.getEntry(prefix + "distance").getDouble(0.0);
    }
    
    /**
     * Gets the estimated distance from a specific robot side to the QR code.
     * Takes into account camera offset and physical camera angle.
     * 
     * @param cameraAngleDegrees The physical angle of the camera tower in degrees.
     *                           0° = camera faces forward (along robot's front)
     *                           90° = camera faces right
     *                           180° = camera faces backward
     *                           270° = camera faces left
     * @return Distance from the specified robot side to the QR code in centimeters
     */
    public double getDistanceFromRobotSide(double cameraAngleDegrees) {
        double cameraToQR = getDistance();
        if (cameraToQR == 0.0) {
            return 0.0;
        }
        
        // Convert camera angle to radians
        double angleRad = Math.toRadians(cameraAngleDegrees);
        
        // Calculate QR position relative to robot center
        // Camera offset in robot frame
        double cameraX = CAMERA_OFFSET_X_CM;
        double cameraY = CAMERA_OFFSET_Y_CM;
        
        // QR position relative to camera (camera looking in its direction)
        double qrRelativeToCameraX = 0;
        double qrRelativeToCameraY = cameraToQR;
        
        // Rotate QR position by camera angle to get it in robot frame
        double qrX = cameraX + (qrRelativeToCameraX * Math.cos(angleRad) - qrRelativeToCameraY * Math.sin(angleRad));
        double qrY = cameraY + (qrRelativeToCameraX * Math.sin(angleRad) + qrRelativeToCameraY * Math.cos(angleRad));
        
        // Determine which side of the robot is closest to the QR code
        // and calculate distance from that side
        double frontDistance = qrY + (ROBOT_LENGTH_CM / 2.0);
        double backDistance = - qrY + (ROBOT_LENGTH_CM / 2.0);
        double rightDistance = qrX + (ROBOT_WIDTH_CM / 2.0);
        double leftDistance = -qrX + (ROBOT_WIDTH_CM / 2.0);
        
        // Return the minimum positive distance (closest side)
        double minDistance = Double.MAX_VALUE;
        if (frontDistance > 0 && frontDistance < minDistance) minDistance = frontDistance;
        if (backDistance > 0 && backDistance < minDistance) minDistance = backDistance;
        if (rightDistance > 0 && rightDistance < minDistance) minDistance = rightDistance;
        if (leftDistance > 0 && leftDistance < minDistance) minDistance = leftDistance;
        
        return minDistance == Double.MAX_VALUE ? 0.0 : minDistance;
    }
    
    /**
     * Gets the estimated distance from the robot's front side to the QR code.
     * 
     * @param cameraAngleDegrees The physical angle of the camera tower in degrees
     * @return Distance from robot front to QR code in centimeters
     */
    public double getDistanceFromFront(double cameraAngleDegrees) {
        double cameraToQR = getDistance();
        if (cameraToQR == 0.0) {
            return 0.0;
        }
        
        double angleRad = Math.toRadians(cameraAngleDegrees);
        
        // QR position in robot frame
        // double qrX = CAMERA_OFFSET_X_CM + (0 * Math.cos(angleRad) - cameraToQR * Math.sin(angleRad));
        double qrY = CAMERA_OFFSET_Y_CM + (0 * Math.sin(angleRad) + cameraToQR * Math.cos(angleRad));
        
        // Distance from front edge of robot to QR
        return qrY + (ROBOT_LENGTH_CM / 2.0);
    }

    public int getTicker() {
        return (int)qrTable.getEntry(prefix + "ticker").getDouble(0);
    }
}
