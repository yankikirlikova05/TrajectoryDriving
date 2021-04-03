package frc.robot;
import java.io.IOException;
import java.nio.file.Path;
import java.util.List;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
//import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
//import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
//import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.Drivetrain;

public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public static AHRS gyro = new AHRS();
  public static Drivetrain drivetrain = new Drivetrain(gyro);
  public static Joystick jstick = new Joystick(Constants.jstick_port);

  public String path = "paths/output/deneme.wpilib.json";
  Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(path);
  Trajectory exampleTrajectory = new Trajectory();

  public RobotContainer() {
    
    configureButtonBindings();
  }

  private void configureButtonBindings() {
    
    drivetrain.setDefaultCommand(
    new RunCommand ( () -> drivetrain.driveMecanum(
      -jstick.getRawAxis(Constants.jstickX), 
      -jstick.getRawAxis(Constants.jstickY), 
      jstick.getRawAxis(Constants.jstickZ)
    ),
    drivetrain)
  );
  }

  public Command getAutonomousCommand() {

    // Create a voltage constraint to ensure we don't accelerate too fast
    var autoVoltageConstraint =
        new DifferentialDriveVoltageConstraint(
            new SimpleMotorFeedforward(Constants.TrajectoryDriving.ksVolts,
                                       Constants.TrajectoryDriving.kvVoltSecondsPerMeter,
                                       Constants.TrajectoryDriving.kaVoltSecondsSquaredPerMeter),
            Constants.TrajectoryDriving.kDriveKinematics,
            12);

    // Create config for trajectory
    TrajectoryConfig config =
        new TrajectoryConfig(Constants.TrajectoryDriving.kMaxSpeedMetersPerSecond,
                             Constants.TrajectoryDriving.kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(Constants.TrajectoryDriving.kDriveKinematics)
            // Apply the voltage constraint
            .addConstraint(autoVoltageConstraint);

    // An example trajectory to follow.  All units in meters.
    try{
      exampleTrajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    }
    catch(IOException ex){
      DriverStation.reportError("Unable to open trajectory" , true);
    }    
    /*exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction

        List.of( 
            new Pose2d(0, 0, new Rotation2d(0)),
            new Pose2d(2.92, 1.3, new Rotation2d(Units.degreesToRadians(0))),
            new Pose2d(5.05, 0.0, new Rotation2d(Units.degreesToRadians(-90))),
            new Pose2d(2.92, -1.3, new Rotation2d(Units.degreesToRadians(-180))),
            new Pose2d(0.0, 0.0, new Rotation2d(Units.degreesToRadians(-180)))
            ),
        // End 3 meters straight ahead of where we started, facing forward
        // Pass config
        config
    );
*/
    RamseteCommand ramseteCommand = new RamseteCommand(
        exampleTrajectory,
        drivetrain::getPose,
        new RamseteController(Constants.TrajectoryDriving.kRamseteB, Constants.TrajectoryDriving.kRamseteZeta),
        new SimpleMotorFeedforward(Constants.TrajectoryDriving.ksVolts,
                                   Constants.TrajectoryDriving.kvVoltSecondsPerMeter,
                                   Constants.TrajectoryDriving.kaVoltSecondsSquaredPerMeter),
                                   Constants.TrajectoryDriving.kDriveKinematics,
        drivetrain::getWheelSpeeds,
        new PIDController(Constants.TrajectoryDriving.kPDriveVelLeft, 0, 0),
        new PIDController(Constants.TrajectoryDriving.kPDriveVelRight, 0, 0),
        // RamseteCommand passes volts to the callback
        drivetrain::tankDriveVolts,
        drivetrain
    );

    // Reset odometry to the starting pose of the trajectory.
    drivetrain.resetOdometry(exampleTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return ramseteCommand.andThen(() -> drivetrain.tankDriveVolts(0, 0));
  }
}