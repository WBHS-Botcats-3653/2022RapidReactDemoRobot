// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.OI;

public class DriveTrain extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */

  //this one will be used for a 
  private DriveTrain driveTrain = null;
  public OI input = OI.getInstance();
  public DifferentialDrive difDrive;
  //right side
  
  public DriveTrain() {
    VictorSP rightDriveFront = new VictorSP(Constants.RFPWMid);
    VictorSP rightDriveBack = new VictorSP(Constants.RBPWMid);
	  MotorControllerGroup rightDrive = new MotorControllerGroup(rightDriveFront, rightDriveBack);
    //left side
    VictorSP leftDriveFront = new VictorSP(Constants.LFPWMid);
    VictorSP leftDriveBack = new VictorSP(Constants.LBPWMid);
	  MotorControllerGroup leftDrive = new MotorControllerGroup(leftDriveFront, leftDriveBack);
	
	  difDrive = new DifferentialDrive(leftDrive, rightDrive);
    leftDrive.setInverted(true);
  }

  //this method initiallizes the drive train
  public DriveTrain getDriveTrain(){
    if(driveTrain == null){
      driveTrain = new DriveTrain();
    }
    
    return driveTrain;
  }

  

  public void testingMotors1(double speed, double rotation){
    getDriveTrain().difDrive.arcadeDrive(speed , rotation);
  }
  /**this method instantiates the arcade drive
   * 
   * you would put this one on robotInit or on tel periodic
   */
  public void ArcadeDrived(){
    getDriveTrain().difDrive.arcadeDrive(input.getThrottle() , input.getSteering());
  }

  public void ArcadeDrived(DriveTrain train){
    train.difDrive.arcadeDrive(input.getThrottle() , input.getSteering());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }


}
