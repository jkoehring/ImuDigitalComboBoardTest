package org.usfirst.frc.team1165.robot.subsystems;

import org.usfirst.frc.team1165.robot.commands.Reporter;
import org.usfirst.frc.team1165.robot.sensors.ADXL345_I2C;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.ITG3200;
import edu.wpi.first.wpilibj.interfaces.Accelerometer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class ImuDigitalComboBoard extends ReportableSubsystem
{
	private ADXL345_I2C accelerometer;
	private ITG3200 gyro;
	
	private final double DEGREES_PER_RADIAN = 180.0 / Math.PI;
	
	public ImuDigitalComboBoard()
	{
		accelerometer = new ADXL345_I2C(I2C.Port.kOnboard, Accelerometer.Range.k16G, true);
		gyro = new ITG3200(I2C.Port.kOnboard, new DigitalInput(0));
	}

	// Put methods for controlling this subsystem
	// here. Call these from Commands.

	public void initDefaultCommand()
	{
		setDefaultCommand(new Reporter(this));
	}
	
	public void report()
	{
		SmartDashboard.putNumber("IMU Accel X", accelerometer.getX());
		SmartDashboard.putNumber("IMU Accel Y", accelerometer.getY());
		SmartDashboard.putNumber("IMU Accel Z", accelerometer.getZ());
		
		SmartDashboard.putNumber("IMU Rate X", gyro.getXGyro().getRate() * DEGREES_PER_RADIAN);
		SmartDashboard.putNumber("IMU Rate Y", gyro.getYGyro().getRate() * DEGREES_PER_RADIAN);
		SmartDashboard.putNumber("IMU Rate Z", gyro.getZGyro().getRate() * DEGREES_PER_RADIAN);
		
		SmartDashboard.putNumber("IMU Angle X", gyro.getXGyro().getAngle() * DEGREES_PER_RADIAN);
		SmartDashboard.putNumber("IMU Angle Y", gyro.getYGyro().getAngle() * DEGREES_PER_RADIAN);
		SmartDashboard.putNumber("IMU Angle Z", gyro.getZGyro().getAngle() * DEGREES_PER_RADIAN);
		
		SmartDashboard.putNumber("IMU Temp", gyro.getTemperature());
	}
	
	public void reset()
	{
		gyro.calibrate(5.0);
		gyro.getXGyro().reset();
		gyro.getYGyro().reset();
		gyro.getZGyro().reset();
	}
}
