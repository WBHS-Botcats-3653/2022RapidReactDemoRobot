/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

//import edu.wpi.first.wpilibj.command.Command;
//import frc.robot.subsystems.Drive;
import edu.wpi.first.wpilibj2.command.CommandBase; 
import frc.robot.OI;
import frc.robot.subsystems.DriveTrain;

public class ArcadeDriveCommand extends CommandBase {
	private OI m_oi;
	DriveTrain m_drivetrain;

	public ArcadeDriveCommand() {
		//super("Drive Control");
		
		m_oi = OI.getInstance();
		m_drivetrain = DriveTrain.getDriveTrain();

		//requires(Drive);
	}

	// Called just before this Command runs the first time
	@Override
	public void initialize() {
}

	// Called repeatedly when this Command is scheduled to run
	@Override
	public void execute() {
		

	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	public boolean isFinished() {
		return false;
	}
	
	// Called once after isFinished returns true
	@Override
	public void end(boolean interrupted) {
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run

	/*
	@Override
	public void interrupted() {
	}
	*/
}
