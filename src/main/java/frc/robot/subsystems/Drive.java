// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.command.Subsystem;

//import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.motorcontrol.VictorSP;

import com.ctre.phoenix.motorcontrol.InvertType;
//import edu.wpi.first.wpilibj.DoubleSolenoid;
//import edu.wpi.first.wpilibj.DoubleSolenoid.Value;


import edu.wpi.first.wpilibj.drive.DifferentialDrive;

//import frc.robot.RobotMap;
import frc.robot.Constants;
import frc.robot.MotorFilter;
import frc.robot.subsystems.Heading;
import frc.robot.commands.ArcadeDriveCommand;

public class Drive extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  private static Drive m_singleton = null;
	private static final double m_wheelDiameter = 6.0; // inches
	private static final int m_countPerRev = 360; // Optical encoder 360 pulse per rev
	private Heading m_heading = null;
  private VictorSP m_leftDriveForward, m_rightDriveForward;
	private VictorSP m_leftDriveBack, m_rightDriveBack;
	private DifferentialDrive m_drive;
	private MotorFilter m_driveFilter;
	private MotorFilter m_turnFilter;

  @Override
	public void initDefaultCommand() {
		setDefaultCommand(new ArcadeDriveCommand());
	}

  public Drive() {
    setName("Drive");

    m_heading = Heading.getInstance();
    //right side
    m_rightDriveForward = new VictorSP(Constants.RFPWMid);
    m_rightDriveBack = new VictorSP(Constants.RBPWMid);

    //left side
    m_leftDriveForward = new VictorSP(Constants.LFPWMid);
    m_leftDriveBack = new VictorSP(Constants.LBPWMid);

    //this one sets the follower and followed
    m_rightDriveBack.follow(m_rightDriveForward);
    m_leftDriveBack.follow(m_leftDriveForward);

    m_leftDriveForward.setInverted(true);
    //m_leftDriveBack.setInverted(InvertType.FollowMaster);


    //this one sets the differential drive
    m_drive =  new DifferentialDrive(m_leftDriveForward, m_rightDriveForward);
    m_drive.setInverted(false);

    //this one is a driving filter, 
    m_driveFilter =  new MotorFilter(10);
	
    m_turnFilter = new MotorFilter(10);


  }
//this one give the angle of the direction
  public double getHeading() {
		return m_heading.getAngle();
	}
  public void arcadeDrive(double xSpeed, double zRotation) {
		m_drive.arcadeDrive(m_driveFilter.update(xSpeed), m_turnFilter.update(zRotation));
	}
//this one get the encoder
/*
	public int getLeftEncoder() {
		return m_leftDriveMaster.getSelectedSensorPosition();
	}

	public int getRightEncoder() {
		return m_rightDriveMaster.getSelectedSensorPosition();
	}

	public double getSpeed() {
		int speedLeft = m_leftDriveMaster.getSelectedSensorVelocity();
		int speedRight = m_rightDriveMaster.getSelectedSensorVelocity();
		double speedAve = (double) (speedLeft + speedRight) / 2.0;
		return speedAve * Math.PI * m_wheelDiameter / 12.0 / m_countPerRev;
	}
*/


	public void reset() {
		m_drive.arcadeDrive(0.0, 0.0);
		m_driveFilter.reset();
		m_turnFilter.reset();
	}

	public static Drive getInstance() {
		if (m_singleton == null) {
			m_singleton = new Drive();
		}
		return m_singleton;
	}
}
