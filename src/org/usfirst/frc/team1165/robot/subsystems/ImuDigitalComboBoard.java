package org.usfirst.frc.team1165.robot.subsystems;

import org.usfirst.frc.team1165.robot.commands.Reporter;
import org.usfirst.frc.team1165.robot.commands.ResetImuDigitalComboBoard;
import org.usfirst.frc.team1165.robot.sensors.ADXL345_I2C;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.ITG3200;
import edu.wpi.first.wpilibj.ITG3200.GyroAxis;
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
	
	private double tempMin;
	private double tempMax;
	
	private GyroAxis xGyro;
	private GyroAxis yGyro;
	private GyroAxis zGyro;
	
	private I2C.Port port;
	
	public ImuDigitalComboBoard(I2C.Port port, DigitalInput interrupt)
	{
		this.port = port;
		
//		accelerometer = new ADXL345_I2C(port, Accelerometer.Range.k16G, true);
		gyro = new ITG3200(port, interrupt, false);
		
		xGyro = gyro.getXGyro();
		yGyro = gyro.getYGyro();
		zGyro = gyro.getZGyro();
		
		tempMin = Double.MAX_VALUE;
		tempMax = Double.MIN_VALUE;
		
	}

	// Put methods for controlling this subsystem
	// here. Call these from Commands.

	public void initDefaultCommand()
	{
		setDefaultCommand(new Reporter(this));
		SmartDashboard.putData("Reset IMU " + port.getValue(), new ResetImuDigitalComboBoard(this));
	}
	
	public void report()
	{
		if (accelerometer != null)
		{
			SmartDashboard.putNumber("IMU " + port.getValue() + " Accel X", accelerometer.getX());
			SmartDashboard.putNumber("IMU " + port.getValue() + " Accel Y", accelerometer.getY());
			SmartDashboard.putNumber("IMU " + port.getValue() + " Accel Z", accelerometer.getZ());
		}
		
		if (gyro != null)
		{
			SmartDashboard.putNumber("IMU " + port.getValue() + " Rate X", xGyro.getRate());
			SmartDashboard.putNumber("IMU " + port.getValue() + " Rate Y", yGyro.getRate());
			SmartDashboard.putNumber("IMU " + port.getValue() + " Rate Z", zGyro.getRate());
			
			SmartDashboard.putNumber("IMU " + port.getValue() + " Angle X", xGyro.getAngle());
			SmartDashboard.putNumber("IMU " + port.getValue() + " Angle Y", yGyro.getAngle());
			SmartDashboard.putNumber("IMU " + port.getValue() + " Angle Z", zGyro.getAngle());
			
			double temp = gyro.getTemperature();
			tempMin = Math.min(temp, tempMin);
			tempMax = Math.max(temp, tempMax);
			
			SmartDashboard.putNumber("IMU " + port.getValue() + " Gyro Temp", temp);
			SmartDashboard.putNumber("IMU " + port.getValue() + " Gyro Temp Min", tempMin);
			SmartDashboard.putNumber("IMU " + port.getValue() + " Gyro Temp Max", tempMax);
			
			SmartDashboard.putNumber("IMU " + port.getValue() + " Gyro Sample Rate", gyro.getSampleRate());
			SmartDashboard.putNumber("IMU " + port.getValue() + " Gyro Cur Sample Rate", gyro.getDynamicSampleRate());
			
			SmartDashboard.putNumber("IMU " + port.getValue() + " Read Error Count", gyro.getReadErrorCount());
			SmartDashboard.putNumber("IMU " + port.getValue() + " Write Error Count", gyro.getWriteErrorCount());
		}
	}
	
	public void reset()
	{
		gyro.resetDynamicSampleRate();
		gyro.calibrate(5.0);
		tempMin = Double.MAX_VALUE;
		tempMax = Double.MIN_VALUE;
	}
}
