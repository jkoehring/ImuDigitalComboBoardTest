package org.usfirst.frc.team1165.robot.subsystems;

import org.usfirst.frc.team1165.robot.commands.Reporter;

import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class OnBoardAccelerometer extends ReportableSubsystem
{
	private BuiltInAccelerometer accelerometer;
	
	public OnBoardAccelerometer()
	{
		accelerometer = new BuiltInAccelerometer();
	}
	
	// Put methods for controlling this subsystem
	// here. Call these from Commands.

	public void initDefaultCommand()
	{
		setDefaultCommand(new Reporter(this));
	}
	
	
	public void report()
	{
		SmartDashboard.putNumber("On-board Accel X", accelerometer.getX());
		SmartDashboard.putNumber("On-board Accel Y", accelerometer.getY());
		SmartDashboard.putNumber("On-board Accel Z", accelerometer.getZ());
	}
}
