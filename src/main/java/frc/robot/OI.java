/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import frc.robot.commands.*;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
  public Joystick _driver = new Joystick(RobotMap.OI_DRIVER_CONTROLLER); 
  public Joystick _operator = new Joystick(RobotMap.OI_OP_CONTROLLER);
  public Joystick _squishy = new Joystick(RobotMap.OI_SQUISHY);

  public Button LADfrontClimb = new JoystickButton(_operator, 1);
  public Button LADrearClimb = new JoystickButton(_operator, 2);
  public Button LADallClimb = new JoystickButton(_operator, 3);
  public Button LADDriveFwd = new JoystickButton(_operator, 4);
  //public Button UpFront = new JoystickButton(_operator, 5);
  //public Button UpBak = new JoystickButton(_operator, 6);

  public OI () {

    LADfrontClimb.whileHeld(new LADFrontUpdate(RobotMap.FRONT_SPEED));
    LADrearClimb.whileHeld(new LADRearUpdate(RobotMap.REAR_SPEED));
    LADallClimb.whileHeld(new LADRearUpdate(RobotMap.REAR_SPEED));
    LADDriveFwd.whileHeld(new LADUpdateDrive(RobotMap.DRIVE_SPEED));

    //RunClimbDown.whileHeld(new climbdown());    
    //RunDriveBack.whileHeld(new climbdriveback());
    //UpFront.whileHeld(new climbupdown());
    //UpBak.whileHeld(new climbbackup());
    
    
  }
}
