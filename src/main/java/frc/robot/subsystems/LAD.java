/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;

import frc.robot.Robot;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.commands.*;
import frc.robot.RobotMap;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.VictorSP;


/**
 * An example subsystem.  You can replace me with your own Subsystem.
 */
public class LAD extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  private CANSparkMax f_motor;
  private CANSparkMax r_motor;
  
  static final double kOffBalanceAngleThresholdDegrees = 10;
  static final double kOonBalanceAngleThresholdDegrees  = 5;
  boolean autoBalanceXMode=false;
  boolean autoBalanceYMode=false;
  VictorSP driveMotor = new VictorSP(RobotMap.LAD_DRIVE);

  @Override
  public void initDefaultCommand() {
    f_motor = new CANSparkMax(RobotMap.LAD_FRONT,MotorType.kBrushless);
    r_motor = new CANSparkMax(RobotMap.LAD_BACK,MotorType.kBrushless);
  
    // Set the default command for a subsystem here.
     //setDefaultCommand(new squishyaxis());
  }


  public void updateFront(double p_val) {
    // Update motor speed to passed in value
    f_motor.set(p_val);
    SmartDashboard.putNumber("Voltage", f_motor.getBusVoltage());
    SmartDashboard.putNumber("Temperature", f_motor.getMotorTemperature());
    SmartDashboard.putNumber("Output", f_motor.getAppliedOutput());
  }

  public void updateRear(double p_val) {
    // Update motor speed to passed in value
    r_motor.set(p_val);
    SmartDashboard.putNumber("Voltage", r_motor.getBusVoltage());
    SmartDashboard.putNumber("Temperature", r_motor.getMotorTemperature());
    SmartDashboard.putNumber("Output", r_motor.getAppliedOutput());
  }

  public void updateAxis() {
    // Update motor speed to passed in value
    double input = Robot.m_oi._operator.getRawAxis(RobotMap.LAD_FRONT);

    updateFront(input);
    updateRear(input);
  }

  public void updateDriveMotor(double p_val) {
    // Update motor speed to passed in value
    driveMotor.set(p_val);
  }

  public void updateAll(double p_front, double p_rear) {
    // Update motor speed to passed in value
    double pitchAngleDegrees    = Robot.ahrs.getPitch();
    if ( !autoBalanceXMode && 
        (Math.abs(pitchAngleDegrees) >= 
        Math.abs(kOffBalanceAngleThresholdDegrees))) {
      autoBalanceXMode = true;
    }
    else if ( autoBalanceXMode && 
            (Math.abs(pitchAngleDegrees) <= 
              Math.abs(kOonBalanceAngleThresholdDegrees))) {
      autoBalanceXMode = false;
    }
    if ( autoBalanceXMode ) {
        double pitchAngleRadians = pitchAngleDegrees * (Math.PI / 180.0);
        p_front = Math.sin(pitchAngleRadians) * -1;
        p_rear = Math.sin(pitchAngleRadians) * -1;
    }
    updateFront(p_front);
    updateRear(p_rear);
  }
}
