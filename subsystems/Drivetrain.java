package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.util.Units;

import com.ctre.phoenix.motorcontrol.NeutralMode;
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
    m_odometry = new DifferentialDriveOdometry(getHeading());
    rearLeftMotor.setNeutralMode(NeutralMode.Coast);
    rearRightMotor.setNeutralMode(NeutralMode.Coast);
    frontLeftMotor.setNeutralMode(NeutralMode.Coast);
    frontRightMotor.setNeutralMode(NeutralMode.Coast);
    frontLeftMotor.setSafetyEnabled(false);
    rearRightMotor.setSafetyEnabled(false);
    frontRightMotor.setSafetyEnabled(false);
    rearLeftMotor.setSafetyEnabled(false);
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
  

 
//Sağ ya da sol - ile çarpılacak
  public static double getLeftDistance(){
    return ((-Drivetrain.frontLeftMotor.getSelectedSensorPosition()/4096.0) * 2.0 * Math.PI * Units.inchesToMeters(3.0) + 
    (-Drivetrain.rearLeftMotor.getSelectedSensorPosition()/4096.0) * 2.0 * Math.PI * Units.inchesToMeters(3.0))/2.0;
  }
  
  public static double getRightDistance(){
    return ((Drivetrain.frontRightMotor.getSelectedSensorPosition()/4096.0) * 2 * Math.PI * Units.inchesToMeters(3.0) + 
    (Drivetrain.rearRightMotor.getSelectedSensorPosition()/4096.0) * 2.0 * Math.PI * Units.inchesToMeters(3.0))/2.0;
  }
    
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_odometry.update(getHeading(), getLeftDistance(), getRightDistance());
  }

  public Rotation2d getHeading(){
    return Rotation2d.fromDegrees(Math.IEEEremainder(-gyro.getAngle(), 360));
    //return Math.IEEEremainder(- gyro.getAngle(),360); 
  }
  //(getHeading().getDegrees()* -1);
//* (DriveMotors.kGyroReversed ? - 1.0 : 1.0);
// ? den sonra - vardı denemek için sildim//

  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    m_odometry.resetPosition(pose, getHeading());
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
    //Sağ ya da sol - ile çarpılacak
    //Adamın biri Sağ önü - ile çarpmış
    frontLeftMotor.setVoltage(leftVolts);
    rearLeftMotor.setVoltage(leftVolts);
    frontRightMotor.setVoltage(-rightVolts);
    rearRightMotor.setVoltage(-rightVolts);
//sağ veya solu eksi ile çarpıcaz
    mecanumDrive.feed();
  }

  public void zeroHeading(){
    gyro.zeroYaw();
  }
  
}
