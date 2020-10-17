/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
//import sun.security.jca.GetInstance;
import frc.robot.OI;
//import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PWMTalonSRX;

/**
 * Add your docs here.
 */
public class DriveTrain extends Subsystem {

  private PWMTalonSRX left = new PWMTalonSRX(RobotMap.leftDrivePort);
  private PWMTalonSRX right = new PWMTalonSRX(RobotMap.rightDrivePort);
  private PWMTalonSRX arm = new PWMTalonSRX(RobotMap.armDrivePort);
  

 // private double leftPowerY, rightPowerY;
 // private double leftPowerX, rightPowerX;

  private double leftPower, rightPower;
  private boolean downArm, upArm;

  public static DriveTrain drive;
  //public Joystick arcJoy;

  public DriveTrain() {
    left.setInverted(false);
    right.setInverted(true);
    arm.setInverted(false);
    //leftEncoder.reset();
    //rightEncoder.reset();
    //gyro.reset();
  }

  public static DriveTrain getInstance() {
    if(drive == null) {
      drive = new DriveTrain();
    }
    return drive;
  }

  public void tankDrive(double leftPow, double rightPow) {
  /* 
    if (leftPow < 0.05 && leftPow > -0.05) {
      leftPow = 0;
    }
    if (rightPow < 0.05 && rightPow > -0.05) {
      rightPow = 0;
    }
*/
    left.set(leftPow);
    right.set(rightPow);
  }
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  @Override
  public void periodic() {
   // leftPowerY = OI.getArcJoy().getY();
   // rightPowerY = OI.getArcJoy().getY();
   // leftPowerX = OI.getArcJoy().getX();
   // rightPowerX = OI.getArcJoy().getX();
    downArm = OI.getArcJoy().getRawButton(3);
    upArm = OI.getArcJoy().getRawButton(4);

    if (downArm) {
      arm.set(-0.2);
    } else if (upArm) {
      arm.set(0.2);
    } else {
      arm.set(0);
    }

      leftPower = OI.getArcJoy().getY();
      rightPower = OI.getArcJoy().getY();
    if (OI.getArcJoy().getX() < -0.2) {
      leftPower += -0.5;
      rightPower += 0.5;
    } else if (OI.getArcJoy().getX() > 0.2) {
      leftPower += 0.5;
      rightPower += -0.5;
    }

    tankDrive(leftPower * 0.3, rightPower * 0.3);
  }
}
