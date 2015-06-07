package org.usfirst.frc.team1165.robot.commands;

import org.usfirst.frc.team1165.robot.Robot;
import org.usfirst.frc.team1165.robot.subsystems.ImuDigitalComboBoard;

import edu.wpi.first.wpilibj.command.Command;


/**
 *
 */
public class ResetImuDigitalComboBoard extends Command
{
	private ImuDigitalComboBoard board;
	public ResetImuDigitalComboBoard(ImuDigitalComboBoard board)
	{
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		this.board = board;
	}

	// Called just before this Command runs the first time
	protected void initialize()
	{
		board.reset();
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute()
	{
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished()
	{
		return true;
	}

	// Called once after isFinished returns true
	protected void end()
	{
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted()
	{
	}
}
