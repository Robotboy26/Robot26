// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package Team4450.Robot25.subsystems;

import static Team4450.Robot25.Constants.alliance;

import java.util.Objects;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import Team4450.Lib.Util;
import Team4450.Robot25.RobotContainer;
import Team4450.Robot25.Constants.DriveConstants;
import gg.questnav.questnav.PoseFrame;
import gg.questnav.questnav.QuestNav;
import Team4450.Robot25.subsystems.LimelightHelpers;

public class QuestNavSubsystem extends SubsystemBase {
  private boolean limelightEnabled = true;
  private boolean allianceRed = false; // Set to true if the robot is on the red alliance
  QuestNav questNav;
   Transform2d ROBOT_TO_QUEST = new Transform2d(-0.32, -0.29, Rotation2d.k180deg); //Original was -0.32, -0.29
  // Transform2d ROBOT_TO_QUEST = new Transform2d(0, 0, Rotation2d.kZero); //Use for characterization
  Pose2d robotPose = DriveConstants.DEFAULT_STARTING_POSE;
  final Pose2d nullPose = new Pose2d(-1, -1, Rotation2d.kZero);

  

  PoseFrame[] poseFrames;

  /** Creates a new QuestNavSubsystem. */
  public QuestNavSubsystem() {
    questNav = new QuestNav();

    resetToZeroPose();

    allianceRed = alliance == Alliance.Red;
  }

  public void resetToZeroPose() {
    Pose2d questPose = robotPose.transformBy(ROBOT_TO_QUEST);
    questNav.setPose(questPose);
    System.out.println("****QRobot reset to zero pose: " + questPose.toString());
  }

  public void resetQuestIMUToAngle(double angle) {
    // questNav.setPose(new Pose2d(getQuestPose().getTranslation(),
    // ROBOT_TO_QUEST.getRotation()
    // .rotateBy(Rotation2d.fromDegrees(angle))));
    System.out.println("Quest Robot Pose: " + 
        java.util.Objects.requireNonNullElse(getQuestRobotPose(),"").toString());
    SmartDashboard.putNumber("QAngle: " , angle);
    System.out.println("New QAngle: " + (Rotation2d.fromDegrees(angle).
        minus(getQuestRobotPose().getRotation())).getDegrees());
    
    Pose2d newRobotPose = new Pose2d(getQuestRobotPose().getTranslation(), Rotation2d.fromDegrees(angle));
    System.out.println(newRobotPose.toString());
    questNav.setPose(newRobotPose.transformBy(ROBOT_TO_QUEST));
  }

  /**
   * Gets the yaw of the QuestNav (Z axis rotation) (yaw is the direction that the
   * questnav is facing around an axis that shoots straight up)
   * 
   * @return
   */
  public double getTrueYaw() {
    return getQuestPose().getRotation().getDegrees(); // With True QuestNav Pose this method returns values in degrees
  }

  public double getQuestRobotYaw() {
    return getQuestRobotPose().getRotation().getDegrees();
  }

  /**
   * Zeroes the yaw of the robot
   * 
   * @return The previous yaw
   */
  public double zeroYaw() {
    double previousYaw = getQuestRobotYaw();
    // System.out.println("Old Yaw: " + previousYaw);
    if (allianceRed) {
      // System.out.println("Yaw 180 " + RobotContainer.isAllianceRed);
      resetQuestIMUToAngle(180);

    } else {
      //System.out.println("Yaw NOT 180 " + RobotContainer.isAllianceRed);
      resetQuestIMUToAngle(0);
    }
    System.out.println("New Yaw: " + getQuestRobotYaw());
    return previousYaw;
  }

  public Pose2d getQuestRobotPose() {

    return (poseFrames != null && poseFrames.length > 0) ?
      poseFrames[poseFrames.length - 1].questPose()
        .transformBy(ROBOT_TO_QUEST.inverse()) :
      nullPose;
    
    // return qPose;
  }

  public double getQTimeStamp() {
    return (poseFrames != null && poseFrames.length > 0) ?
      poseFrames[poseFrames.length - 1].dataTimestamp() :
      0;
  }

  public double getQAppTimeStamp() {
    return (poseFrames != null && poseFrames.length > 0) ?
        poseFrames[poseFrames.length - 1].appTimestamp() :
        0;
  }

  public Pose2d getQuestPose() {
    return (poseFrames != null && poseFrames.length > 0) ?
        poseFrames[poseFrames.length - 1].questPose() :
        nullPose;
  }

  public void resetQuestOdometry(Pose2d rP) {

    // Transform by the offset to get the Quest pose
    Pose2d questPose = rP.transformBy(ROBOT_TO_QUEST);

    // Send the reset operation
    questNav.setPose(questPose);
    System.out.println("Quest Odometry Reset To: " + questPose.toString());
    System.out.println("QRP: " + rP.toString());
  }

  @Override
  public void periodic() {
      // https://github.com/LimelightVision/limelight-examples/tree/main/java-wpilib/swerve-megatag-odometry
      // https://docs.limelightvision.io/docs/docs-limelight/getting-started/summary
      SmartDashboard.putBoolean("LimelightEnabled: ", limelightEnabled);
      if (limelightEnabled) {
          LimelightHelpers.PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");
          if (mt1.tagCount >= 2) {  // Only trust measurement if we see multiple tags
              RobotContainer.driveBase.updateOdometryLimelight(mt1.pose, mt1.timestampSeconds);
              Util.consoleLog("Because the limelight sees 2 or more tags with will set the robot position with the limelight position.");
          }
      }

    if (questNav.isTracking()) {
      // This method will be called once per scheduler run
      SmartDashboard.putString("qTranformedPose: ", getQuestRobotPose().toString());
      SmartDashboard.putString("qTruePose: ", getQuestPose().toString());
      SmartDashboard.putNumber("TimeStamp: ", getQTimeStamp());
      SmartDashboard.putNumber("TimeStampA: ", getQAppTimeStamp());
      SmartDashboard.putNumber("TimeStampFPGS: ", (getQTimeStamp()));
      SmartDashboard.putString("Is Tracking", questNav.isTracking() ? "True" : "False");
      questNav.commandPeriodic();

      //update pose Frames
      poseFrames = questNav.getAllUnreadPoseFrames();
      // Display number of frames provided
      SmartDashboard.putNumber("qFrames", poseFrames.length);
      if(DriveConstants.ODOMETRY_UPDATE_FROM_QUESTNAV) {
        for (PoseFrame questFrame : poseFrames) {
          
          // Get the pose of the Quest
          // Pose2d questPose = questFrame.questPose();
          // Get timestamp for when the data was sent
          double timestamp = questFrame.dataTimestamp();

          // Transform by the mount pose to get your robot pose
          // Pose2d robotPose = questPose.transformBy(ROBOT_TO_QUEST.inverse());

          // You can put some sort of filtering here if you would like!

          // Add the measurement to our estimator
          RobotContainer.driveBase.updateOdometryQuest(getQuestRobotPose(), timestamp);

        
        }
      }
    }
  }
}
