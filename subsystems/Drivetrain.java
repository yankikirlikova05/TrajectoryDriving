package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveMotors;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.util.Units;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

public class Drivetrain extends SubsystemBase {
  /** Creates a new Drivetrain. */

  //kinematics
  //odometry
  //position
  

  public static WPI_TalonSRX frontLeftMotor = new WPI_TalonSRX(Constants.frontLeftMotor);
  public static WPI_TalonSRX frontRightMotor = new WPI_TalonSRX(Constants.frontRightMotor);
  public static WPI_TalonSRX rearLeftMotor = new WPI_TalonSRX(Constants.rearLeftMotor);
  public static WPI_TalonSRX rearRightMotor = new WPI_TalonSRX(Constants.rearRightMotor);
  public MecanumDrive mecanumDrive = new MecanumDrive(frontLeftMotor, frontRightMotor, rearLeftMotor, rearRightMotor);
  private AHRS gyro;
  private final DifferentialDriveOdometry m_odometry;



  public Drivetrain(AHRS gyro) {
    this.gyro = gyro;
    m_odometry = new DifferentialDriveOdometry(gyro.getRotation2d());
  }

  public void driveMecanum(double x, double y, double rot){
    mecanumDrive.driveCartesian(y,x,rot);
    
  }

  public void resetEncoders(){
    frontRightMotor.setSelectedSensorPosition(0);
    frontLeftMotor.setSelectedSensorPosition(0);
    rearLeftMotor.setSelectedSensorPosition(0);
    rearRightMotor.setSelectedSensorPosition(0);
    
  }

  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }
  

  /* motor sürücüleri üzerinden encoder datası alma{
    getLeftDistance
    getRightDistance

    resetEncoders
    
    getAverageDistance
  }*/

//drivetrain 
  public static double getLeftDistance(){
    return ((-Drivetrain.frontLeftMotor.getSelectedSensorPosition()/4096.0) * 2.0 * Math.PI * (3.0 * 2.54) + 
    (-Drivetrain.rearLeftMotor.getSelectedSensorPosition()/4096.0) * 2.0 * Math.PI * (3.0 * 2.54))/2.0;
  }
  
  public static double getRightDistance(){
    return ((Drivetrain.frontRightMotor.getSelectedSensorPosition()/4096.0) * 2 * Math.PI * (3.0 * 2.54) + 
    (Drivetrain.rearRightMotor.getSelectedSensorPosition()/4096.0) * 2.0 * Math.PI * (3.0 * 2.54))/2.0;
  }
    
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_odometry.update(gyro.getRotation2d(), getLeftDistance(), getRightDistance());
  }

  public double getHeading(){
    return Math.IEEEremainder(gyro.getAngle(), 360) * (DriveMotors.kGyroReversed ? -1.0 : 1.0);
  }

  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    m_odometry.resetPosition(pose, gyro.getRotation2d());
  }

  public double getAverageDistance(){
    return (getLeftDistance() + getRightDistance()) / 2.0;
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    //METER PER SECOND OLARAK İSTİYO
    double circumference = Units.inchesToMeters(3) * Math.PI * 2;

    return new DifferentialDriveWheelSpeeds(frontLeftMotor.getSelectedSensorVelocity()* (10.0 / 4096.0) * circumference, frontRightMotor.getSelectedSensorVelocity()* (10.0 / 4096.0) * circumference);
    /*
    leftMaster.getSelectedSensorVelocity() * (10.0 / 4096) * wheelCurcumference,
        rightMaster.getSelectedSensorVelocity() * (10.0 / 4096) * wheelCurcumference
    */
  }

  public void tankDriveVolts(double leftVolts, double rightVolts) {
    frontLeftMotor.setVoltage(leftVolts);
    rearLeftMotor.setVoltage(leftVolts);
    frontRightMotor.setVoltage(-rightVolts);
    rearRightMotor.setVoltage(-rightVolts);

    mecanumDrive.feed();
  }

  public void zeroHeading(){
    gyro.zeroYaw();
  }
  
}
