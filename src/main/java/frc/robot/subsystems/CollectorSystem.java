// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CollectorSystem extends SubsystemBase {

  CANSparkMax collectorMotor;
  Solenoid collectorSolenoid;
 
  public CollectorSystem() {
    collectorMotor = new CANSparkMax(10, MotorType.kBrushless);
    collectorMotor.setInverted(false);
    collectorSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, 6);
  }
 
  public void CollectorFunction(){
    collectorMotor.set(1);
  }

  public void StopCollect() {
    collectorMotor.stopMotor();
  }
  
  public void BallPullOut(){
    collectorMotor.set(-1);
  }

  /*public boolean CompressorStatus(){
    return CollectorCompressor.enabled();
  }

  public boolean CompressorSwitchStatus(){
    return CollectorCompressor.getPressureSwitchValue();
  }

  public double CompressorCurrentConsumed(){
    return CollectorCompressor.getCurrent();
  }*/

  public void StartCollectorSolenoid(){
    collectorSolenoid.set(true);
  }

  public void StoptCollectorSolenoid(){
    collectorSolenoid.set(false);
  }
}
