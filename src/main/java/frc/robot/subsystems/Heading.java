/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.ADIS16470_IMU;

/**
 * Wrapper class to gyro on Roborio.
 */
public class Heading {
	private static Heading m_singleton;
	private ADIS16470_IMU m_gyro;

	private Heading() {
		m_gyro = new ADIS16470_IMU();
		m_gyro.calibrate();
	}

	public Sendable getGyro() {
		return m_gyro;
	}

	public double getAngle() {
		return m_gyro.getAngle();
	}

	public static Heading getInstance() {
		if (m_singleton == null) {
			m_singleton = new Heading();
		}
		return m_singleton;
	}
}
