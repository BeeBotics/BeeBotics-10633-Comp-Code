// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

//package frc.robot.subsystems.swervedrive;
//import org.littletonrobotics.junction.LoggedRobot;
import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.CoordinateSystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;

import com.pathplanner.lib.config.RobotConfig;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;

import java.io.File;
import java.util.List;
import java.util.function.BooleanSupplier;

import static edu.wpi.first.units.Units.Meter;

import com.ctre.phoenix6.swerve.jni.SwerveJNI.ModuleState;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.swerve.SwerveSetpoint;
import com.pathplanner.lib.util.swerve.SwerveSetpointGenerator;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import swervelib.SwerveDrive;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveControllerConfiguration;
import swervelib.parser.SwerveDriveConfiguration;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {

  SwerveDrive swerveDrive;
  private final SwerveDriveKinematics kinematics;
  private final SimSwerveModule[] modules;

  private static final NetworkTableInstance NTinstance = NetworkTableInstance.getDefault();
  private static final NetworkTable TableOut = NTinstance.getTable("table");
  private final StructPublisher<Pose2d> publishRobotPose;
  private final StructArrayPublisher<SwerveModuleState> moduleStatePublisher;
  private final StructPublisher<ChassisSpeeds> chassisSpeedPublisher;
  private final DoublePublisher gyroPublisher;
  private final StructPublisher<Pose2d> megaTag1Publisher;

  // Create MAXSwerveModules
  private final MAXSwerveModule m_frontLeft = new MAXSwerveModule(
      DriveConstants.kFrontLeftDrivingCanId,
      DriveConstants.kFrontLeftTurningCanId,
      DriveConstants.kFrontLeftChassisAngularOffset);

  private final MAXSwerveModule m_frontRight = new MAXSwerveModule(
      DriveConstants.kFrontRightDrivingCanId,
      DriveConstants.kFrontRightTurningCanId,
      DriveConstants.kFrontRightChassisAngularOffset);

  private final MAXSwerveModule m_rearLeft = new MAXSwerveModule(
      DriveConstants.kRearLeftDrivingCanId,
      DriveConstants.kRearLeftTurningCanId,
      DriveConstants.kBackLeftChassisAngularOffset);

  private final MAXSwerveModule m_rearRight = new MAXSwerveModule(
      DriveConstants.kRearRightDrivingCanId,
      DriveConstants.kRearRightTurningCanId,
      DriveConstants.kBackRightChassisAngularOffset);

  // The gyro sensor
  private final AHRS m_gyro = new AHRS(NavXComType.kMXP_SPI);

  private final SwerveDrivePoseEstimator m_poseEstimator;
  // Odometry class for tracking robot pose
  SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(
      DriveConstants.kDriveKinematics,
      Rotation2d.fromDegrees(-m_gyro.getAngle()),
      new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
      });
  // Basic targeting data
  double tx = LimelightHelpers.getTX(""); // Horizontal offset from crosshair to target in degrees
  double ty = LimelightHelpers.getTY(""); // Vertical offset from crosshair to target in degrees
  double ta = LimelightHelpers.getTA(""); // Target area (0% to 100% of image)
  boolean hasTarget = LimelightHelpers.getTV(""); // Do you have a valid target?

  double txnc = LimelightHelpers.getTXNC(""); // Horizontal offset from principal pixel/point to target in degrees
  double tync = LimelightHelpers.getTYNC(""); // Vertical offset from principal pixel/point to target in degrees

  private double gyro;

  private Pose3d tagInRobotFrame;

  private Pose3d leftTargetBranchPose;
  private Pose3d rightTargetBranchPose;

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    setupPathPlanner(); // Call your AutoBuilder configuration here.
    // Usage reporting for MAXSwerve template
    HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_MaxSwerve);
    modules = new SimSwerveModule[] {
        new SimSwerveModule(),
        new SimSwerveModule(),
        new SimSwerveModule(),
        new SimSwerveModule()
    };

    kinematics = Constants.DriveConstants.kDriveKinematics;
    // kinematics = new SwerveDriveKinematics(
    // Constants.Swerve.flModuleOffset,
    // Constants.Swerve.frModuleOffset,
    // Constants.Swerve.blModuleOffset,
    // Constants.Swerve.brModuleOffset
    // );
    publishRobotPose = TableOut.getStructTopic("RobotPose", Pose2d.struct).publish();
    moduleStatePublisher = TableOut.getStructArrayTopic("Module States", SwerveModuleState.struct).publish();
    chassisSpeedPublisher = TableOut.getStructTopic("Chassis Speeds", ChassisSpeeds.struct).publish();
    gyroPublisher = TableOut.getDoubleTopic("Gyro").publish();
    megaTag1Publisher = TableOut.getStructTopic("MegaTag1", Pose2d.struct).publish();


    m_poseEstimator = new SwerveDrivePoseEstimator(
        DriveConstants.kDriveKinematics,
        m_gyro.getRotation2d(),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        },
        Pose2d.kZero,
        VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)),
        VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30)));

  }

  @Override
  public void periodic() {
    gyro = -m_gyro.getAngle();

    // Update the odometry in the periodic
    m_odometry.update(
        Rotation2d.fromDegrees(gyro),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        });

    m_poseEstimator.update(
        Rotation2d.fromDegrees(gyro),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        });

    // double robotYaw = m_gyro.getYaw();
    // LimelightHelpers.PoseEstimate limelightMeasurementMT1;
    // // // Get the pose estimate
    // if (isBlueAlliance().getAsBoolean()) {
    //   limelightMeasurementMT1 = LimelightHelpers.getBotPoseEstimate_wpiBlue("");
    // } else {
    //   limelightMeasurementMT1 = LimelightHelpers.getBotPoseEstimate_wpiRed("");
    // }


    // publishRobotPose.set(getPose());
    // gyroPublisher.set(gyro);
    // megaTag1Publisher.set(limelightMeasurementMT1.pose);
    

  //   if(LimelightHelpers.getTV("")) {
  //   var tagInRobotFrameEDN = LimelightHelpers.getTargetPose3d_RobotSpace("");
  //  // System.out.print("Pose= " + tagInRobotFrameEDN.getX() + ", " + tagInRobotFrameEDN.getY() + ", " + tagInRobotFrameEDN.getZ()); 
  //   // // The X is the side to side distance from the tag, the Z is the forward and backward
  //   // System.out.println(" Dist= " + limelightMeasurementMT1.rawFiducials[0].distToRobot);
  //   if(tagInRobotFrameEDN.getZ() <= 1.5) {
  //     tagInRobotFrame = CoordinateSystem.convert(tagInRobotFrameEDN, CoordinateSystem.EDN(), CoordinateSystem.NWU());
  //     tagInRobotFrame = new Pose3d(tagInRobotFrame.getTranslation(), new Rotation3d(
  //       - tagInRobotFrame.getRotation().getY(),
  //       Math.PI/2. + tagInRobotFrame.getRotation().getX(),
  //       Math.PI/2. - tagInRobotFrame.getRotation().getZ() > 0. ? 
  //         Math.PI + Math.PI/2. - tagInRobotFrame.getRotation().getZ() : 
  //         - tagInRobotFrame.getRotation().getZ()));
  //     leftTargetBranchPose = CoordinateSystem.convert(new Pose3d(0.16, -0.3, 0.62, Rotation3d.kZero), CoordinateSystem.EDN(), CoordinateSystem.NWU());
  //     // // Add it to your pose estimator
  //     //System.out.println(" Tag Converted= " + tagInRobotFrame);
  //     // System.out.format(" NWU R adjust T:%.3f, %.3f, %.3f, R:%.3f, %.3f, %.3f%n%n",
  //     // tagInRobotFrame.getTranslation().getX(),
  //     // tagInRobotFrame.getTranslation().getY(),
  //     // tagInRobotFrame.getTranslation().getZ(),
  //     // Units.radiansToDegrees(tagInRobotFrame.getRotation().getX()), 
  //     // Units.radiansToDegrees(tagInRobotFrame.getRotation().getY()), 
  //     // Units.radiansToDegrees(tagInRobotFrame.getRotation().getZ())); 
  //    // System.out.println(" left Target Branch Converted= " + leftTargetBranchPose);

  //     if (DriverStation.isAutonomousEnabled()) {
  //       // System.out.println("Auto Add Vision");
  //       m_poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.5, .5, 9999999));
  //       m_poseEstimator.addVisionMeasurement(
  //           limelightMeasurementMT1.pose,
  //           limelightMeasurementMT1.timestampSeconds);
  //     }
  //     if (DriverStation.isTeleopEnabled()) {
  //       //resetOdometry(tagInRobotFrame.toPose2d());
  //     }

  //     }
      
    // }
    // x = 0.62, y = -0.144 z = 0.302 Left reef post
  }
  public Pose2d getLeftBranchPose() {
    return new Pose2d(0.62, -0.144, new Rotation2d());
  }
  public Pose2d getCurrentPose() {
    return new Pose2d(tagInRobotFrame.getTranslation().getX(), 
        tagInRobotFrame.getTranslation().getY(), 
        new Rotation2d(tagInRobotFrame.getRotation().getZ()));
  }
  public Command driveToPositionCommand(Pose2d targetPose, Pose2d currentPose) {
    
    // Create a list of two waypoints representing the path to follow,
    // from the current pose to the target pose
    List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
                                new Pose2d(currentPose.getTranslation(), currentPose.getRotation()),
                                new Pose2d(targetPose.getTranslation(), targetPose.getRotation()));
        
    // Create the constraints of the path to be followed
    PathConstraints constraints = new PathConstraints(
      2.8, 2.8, // these two are different instances of max speeds, you can make them the same.
      2 * Math.PI, 2 * Math.PI);
    // Calculate the robot's current velocity and rotation to set as the ideal starting state of the path
    // double vxMetersPerSecond = getState().Speeds.vxMetersPerSecond;
    // double vyMetersPerSecond = getState().Speeds.vyMetersPerSecond;
    // double velocity = Math.sqrt(vxMetersPerSecond * vxMetersPerSecond + vyMetersPerSecond * vyMetersPerSecond);
    // Rotation2d rotation = getPose().getRotation();
    // IdealStartingState idealStartingState = new IdealStartingState(velocity, rotation);
    IdealStartingState idealStartingState = new IdealStartingState(0, Rotation2d.kZero);
    // Create the new path using the waypoints
    PathPlannerPath path = new PathPlannerPath(
                                waypoints,
                                constraints,
                                idealStartingState, // set this to null if not working
                                new GoalEndState(0.0, targetPose.getRotation()));
    path.preventFlipping = true;

    // Create the path following command
    return AutoBuilder.followPath(path);
}

  public void setGyro(double angle) {
    m_gyro.setAngleAdjustment(angle);
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
    // return m_poseEstimator.getEstimatedPosition();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(
        Rotation2d.fromDegrees(-m_gyro.getAngle()),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        },
        pose);
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    // Convert the commanded speeds into the correct units for the drivetrain
    double xSpeedDelivered = xSpeed * DriveConstants.kMaxSpeedMetersPerSecond;
    double ySpeedDelivered = ySpeed * DriveConstants.kMaxSpeedMetersPerSecond;
    double rotDelivered = rot * DriveConstants.kMaxAngularSpeed;

    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered,
                Rotation2d.fromDegrees(-m_gyro.getAngle()))
            : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);

    moduleStatePublisher.set(swerveModuleStates);
  }

  /**
   * Sets the wheels into an X formation to prevent movement.
   */
  public void setX() {
    m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_rearLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearRight.resetEncoders();
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_gyro.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return Rotation2d.fromDegrees(-m_gyro.getAngle()).getDegrees();
  }

  /**
   * Gets the current field-relative velocity (x, y and omega) of the robot
   *
   * @return A ChassisSpeeds object of the current field-relative velocity
   */

  public ChassisSpeeds getFieldVelocity() {
    return swerveDrive.getFieldVelocity();
  }

  public void setStates(SwerveModuleState[] targetStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(targetStates, Constants.Swerve.maxModuleSpeed);

    for (int i = 0; i < modules.length; i++) {
      modules[i].setTargetState(targetStates[i]);
    }
  }

  class SimSwerveModule {
    private SwerveModulePosition currentPosition = new SwerveModulePosition();
    private SwerveModuleState currentState = new SwerveModuleState();

    public SwerveModulePosition getPosition() {
      return currentPosition;
    }

    public SwerveModuleState getState() {
      return currentState;
    }

    public void setTargetState(SwerveModuleState targetState) {
      currentState = SwerveModuleState.optimize(targetState, currentState.angle);

      currentPosition = new SwerveModulePosition(
          currentPosition.distanceMeters + (currentState.speedMetersPerSecond * 0.02), currentState.angle);
    }
  }

  /**
   * Gets the current velocity (x, y and omega) of the robot
   *
   * @return A {@link ChassisSpeeds} object of the current velocity
   */

  public ChassisSpeeds getRobotVelocity() {
    return swerveDrive.getRobotVelocity();
  }

  public ChassisSpeeds getRobotRelativeSpeeds() {
    return Constants.DriveConstants.kDriveKinematics.toChassisSpeeds(m_frontLeft.getState(),
        m_frontRight.getState(),
        m_rearLeft.getState(),
        m_rearRight.getState());
  }

  private void driveRobotRelative(ChassisSpeeds speeds) {
    drive(speeds, false);
  }

  private void drive(ChassisSpeeds speeds, boolean fieldRelative) {

    if (fieldRelative)
      speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, getPose().getRotation());
    speeds = ChassisSpeeds.discretize(speeds, .02); // LoggedRobot.defaultPeriodSecs
    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
    setModuleStates(swerveModuleStates);

    moduleStatePublisher.set(swerveModuleStates);
    chassisSpeedPublisher.set(speeds);
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return -m_gyro.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }

  public void setupPathPlanner() {

    RobotConfig config;
    try {
      config = RobotConfig.fromGUISettings(); // needs to be in a try/catch

      final boolean enableFeedforward = true;

      // Configure AutoBuilder last. Other sources say you can put this in your
      // DriveSubsystem but I had errors doing that.
      AutoBuilder.configure(
          this::getPose,
          this::resetOdometry,
          this::getRobotRelativeSpeeds,
          (speeds, feedforwards) -> driveRobotRelative(speeds),
          new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for
                                          // holonomic drive trains
              new PIDConstants(0.04, 0.0, 0.0), // Translation PID constants
              new PIDConstants(1.0, 0.0, 0.0) // Rotation PID constants (i usually match these to teleop driving
                                              // constants.)
          ),
          config,
          // () -> {
          // // Boolean supplier that controls when the path will be mirrored for the red
          // // alliance
          // // This will flip the path being followed to the red side of the field.
          // // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

          // var alliance = DriverStation.getAlliance();
          // if (alliance.isPresent()) {
          // return alliance.get() == DriverStation.Alliance.Blue;
          // }
          // return false;
          // },
          isBlueAlliance(),
          this
      // Reference to this subsystem to set requirements
      );

    } catch (Exception e) {
      e.printStackTrace();
    }

    // Uncomment this if you want to monitor the configuration confirmation.
    // System.out.println(AutoBuilder.isConfigured());

  }

  public BooleanSupplier isBlueAlliance() {
    return () -> {
      // Boolean supplier that controls when the path will be mirrored for the red
      // alliance
      // This will flip the path being followed to the red side of the field.
      // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

      var alliance = DriverStation.getAlliance();
      if (alliance.isPresent()) {
        return alliance.get() == DriverStation.Alliance.Blue;
      }
      return false;
    };
  }

  public Command getAutonomousCommand(String pathName) {
    return new PathPlannerAuto(pathName);
  }

  public Command driveToPose(Pose2d pose) {

    PathConstraints constraints = new PathConstraints(
        2.8, 2.8, // these two are different instances of max speeds, you can make them the same.
        2 * Math.PI, 2 * Math.PI); // same for these two angular speeds

    // Since AutoBuilder is configured, we can use it to build pathfinding commands
    return AutoBuilder.pathfindToPose(
        pose,
        constraints,
        edu.wpi.first.units.Units.MetersPerSecond.of(0) // Goal end velocity in meters/sec
    );
  }
}
