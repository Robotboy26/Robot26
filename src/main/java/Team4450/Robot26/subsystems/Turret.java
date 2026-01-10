package Team4450.Robot26.subsystems;

import Team4450.Robot26.Constants;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class Turret extends SubsystemBase {
    public Turret() {

    }

    @Override
    public void periodic() {
        // What will the turret need to update over time
    }

    public void aimTurret(Pose2d robotPosition) {
        setTargetAngle(getAngleToFaceGoalDegrees(robotPosition));

        double targetFlywheelSpeed = getNeededFlywheelSpeed(distToGoal(robotPosition));
        setFlywheelSpeed(targetFlywheelSpeed);
    }

    public void setTargetAngle(double angleInSomething) {
        // Set the hardware angle for the turret
    }

    public void setFlywheelSpeed(double targetFlywheelSpeed) {
        // Use a PIDF to keep the flywheel speed at the target
    }

    public double getNeededFlywheelSpeed(double distToGoal) {
        double targetVelocity = interpolateFlywheelSpeedByDistance(distToGoal);
        return targetVelocity * Constants.FLYWHEEL_MAX_THEORETICAL_RPM; // Normalize the target velocity by the max theoretical
    }

    public double distToGoal(Pose2d robotPosition) {
        // If blue side
        double xDiff = 0;
        double yDiff = 0;
        if (Constants.alliance == Alliance.Blue) {
            xDiff = Constants.GOAL_BLUE_X - robotPosition.getX();
            yDiff = Constants.GOAL_BLUE_Y + robotPosition.getY();
        // If red side
        } else if (Constants.alliance == Alliance.Red) {
            xDiff = Constants.GOAL_RED_X + robotPosition.getX();
            yDiff = Constants.GOAL_RED_Y - robotPosition.getY();
        } else {
            // Error
        }
        return Math.sqrt(Math.pow(xDiff, 2) + Math.pow(yDiff, 2));
    }

    public double getAngleToFaceGoalDegrees(Pose2d robotPosition) {
        // If blue side
        double xDiff = 0;
        double yDiff = 0;
        if (Constants.alliance == Alliance.Blue) {
            xDiff = Constants.GOAL_BLUE_X - robotPosition.getX();
            yDiff = Constants.GOAL_BLUE_Y + robotPosition.getY();
        // If red side
        } else if (Constants.alliance == Alliance.Red) {
            xDiff = Constants.GOAL_RED_X + robotPosition.getX();
            yDiff = Constants.GOAL_RED_Y - robotPosition.getY();
        } else {
            // Error
        }
        // Simplify this stuff
        return (Math.toDegrees(Math.atan(yDiff / xDiff)) + robotPosition.getRotation().getDegrees() - 180);
    }

    public double interpolateFlywheelSpeedByDistance(double distToGoal) {
        double lowerPoint = Constants.FLYWHEEL_SPEED_DISTANCE_TABLE[0];

        int lowerPointIndex = 0;

        double higherPoint = Constants.FLYWHEEL_SPEED_DISTANCE_TABLE[Constants.FLYWHEEL_SPEED_DISTANCE_TABLE.length - 1];
        int higherPointIndex = Constants.FLYWHEEL_SPEED_DISTANCE_TABLE.length - 1;

        double currentDistance;
        for (int i = Constants.FLYWHEEL_SPEED_DISTANCE_TABLE.length - 2; i > 0; i--){
            currentDistance = Constants.FLYWHEEL_SPEED_TABLE[i];
            if(currentDistance > distToGoal){
                if (currentDistance < higherPoint) {
                    higherPoint = currentDistance;
                    higherPointIndex = i;
                }
            }else if (currentDistance < distToGoal){
                if (currentDistance >= lowerPoint) {
                    lowerPoint = currentDistance;
                    lowerPointIndex = i;
                }
            }else if (currentDistance == distToGoal){
                return Constants.FLYWHEEL_SPEED_TABLE[i];
            }
        }
        double lowerSpeed = Constants.FLYWHEEL_SPEED_TABLE[lowerPointIndex];
        double higherSpeed = Constants.FLYWHEEL_SPEED_TABLE[higherPointIndex];

        return linearInterpolate(lowerSpeed, higherSpeed, (distToGoal - lowerPoint) / (higherPoint - lowerPoint));
    }

    public static double linearInterpolate(double point1, double point2, double percentageSplit) {
        return point1 + ((point2 - point1) * percentageSplit);
    }
}
