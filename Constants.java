package frc.robot;

import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final class Test999 {
		public static final boolean kGyroReversed = true;
		public static final double ksVolts = 0;
		public static final double kvVoltSecondsPerMeter = 0;
		public static final double kaVoltSecondsSquaredPerMeter = 0;

		public static final double kTrackwidthMeters = 0.65;

    }

	public static final int jstick_port = 0;

	public static final int jstickY = 1;

    public static final int jstickX = 0;

    public static final int jstickZ = 2;

	//Device number yazılacak
	public static int frontLeftMotor = 4;
    public static int frontRightMotor = 2;
    public static int rearLeftMotor = 3;
    public static int rearRightMotor = 1;

	public static final class TrajectoryDrivingEski{
		//Eski değer; 0.91 -characterizationdan
		public static final double ksVolts = 3.33;
		//Eski değer 3.66
		public static final double kvVoltSecondsPerMeter = 3.57;
		public static final double kaVoltSecondsSquaredPerMeter = 0.068;

		// Example value only - as above, this must be tuned for your drive!
		//0.839 eski değer
		public static final double kPDriveVelLeft = 0.0;
		public static final double kPDriveVelRight = 0.0;
		//public static final double kTrackwidthMeters = 0.7;
		//public static final double kTrackwidthMeters = 1.2370455376;
		public static final double kTrackwidthMeters = 1.16831373;
		public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(kTrackwidthMeters);

    	public static final double kMaxSpeedMetersPerSecond = 3;
    	public static final double kMaxAccelerationMetersPerSecondSquared = 3;
		public static final double kRamseteB = 2;
    	public static final double kRamseteZeta = 0.7;
	}

	public static final class TrajectoryDrivingWithTurret{
		public static final double ksVolts = 1.11;
		//Eski değer 3.66
		public static final double kvVoltSecondsPerMeter = 3.51;
		public static final double kaVoltSecondsSquaredPerMeter = 0.0327;

		// Example value only - as above, this must be tuned for your drive!
		//0.173 eski değer
		public static final double kPDriveVelLeft = 0;
		public static final double kPDriveVelRight = 0;
		//public static final double kTrackwidthMeters = 0.7;
		//public static final double kTrackwidthMeters = 1.2370455376;
		//public static final double kTrackwidthMeters = 1.2135815005626045;
		public static final double kTrackwidthMeters = 1.2135815005626045;

		public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(kTrackwidthMeters);

    	public static final double kMaxSpeedMetersPerSecond = 3;
    	public static final double kMaxAccelerationMetersPerSecondSquared = 3;
		public static final double kRamseteB = 2;
    	public static final double kRamseteZeta = 0.7;

	}

	public static final class TrajectoryDriving{
		//0.765
		public static final double ksVolts = 0.765 * 3.2;
		//Eski değer 3.58a
		public static final double kvVoltSecondsPerMeter = 3.58 * 0.6;
		// 0.034
		public static final double kaVoltSecondsSquaredPerMeter = 0.034 * 2;

		// Example value only - as above, this must be tuned for your drive!
		//0.173 eski değer, 

		public static final double kPDriveVelLeft = 0.0172;
		public static final double kPDriveVelRight = 0.0172;
		//public static final double kPDriveVelLeft = 0.25;
		//public static final double kPDriveVelRight = 0.25;

		//public static final double kTrackwidthMeters = 0.7;
		//public static final double kTrackwidthMeters = 1.2370455376;
		//public static final double kTrackwidthMeters = 1.2135815005626045;
		//public static final double kTrackwidthMeters = 1.349615199426128;
		public static final double kTrackwidthMeters = 1.349615199426128 * 1;


		public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(kTrackwidthMeters);

    	public static final double kMaxSpeedMetersPerSecond = 3;
    	public static final double kMaxAccelerationMetersPerSecondSquared = 3;
		public static final double kRamseteB = 2;
    	public static final double kRamseteZeta = 0.7;

	}
}