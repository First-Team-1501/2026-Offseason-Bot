// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.supurdueper.robot2026.subsystems;

import java.util.concurrent.PriorityBlockingQueue;

import org.supurdueper.lib.subsystems.SupurdueperSubsystem;
import org.supurdueper.robot2026.CanId;
import org.supurdueper.robot2026.Constants;
import org.supurdueper.robot2026.Robot;

import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.controls.ColorFlowAnimation;
import com.ctre.phoenix6.controls.FireAnimation;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.controls.TwinkleAnimation;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.RGBWColor;
import com.ctre.phoenix6.signals.StripTypeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Lights extends SubsystemBase implements SupurdueperSubsystem {
  /** Creates a new Lights. */
  private CANdle candle;

  private CANdleConfiguration candleConfig;

  public Lights() {
    super();
    candle = new CANdle(CanId.CANDLE.getDeviceNumber());
    candleConfig = new CANdleConfiguration();
    candleConfig.LED.StripType = StripTypeValue.GRB;
    candleConfig.LED.BrightnessScalar = Constants.LightsConstants.brightness;
    candle.getConfigurator().apply(candleConfig);
    Robot.add(this);
  }

  private void setLED(int r, int g, int b){
    candle.setControl(new SolidColor(0, Constants.LightsConstants.LEDCount)
    .withColor(new RGBWColor(r, g, b)));
  }

    private void twinkleLED(int r,int g,int b){
    candle.setControl(new TwinkleAnimation(0 ,Constants.LightsConstants.LEDCount)
    .withColor(new RGBWColor(r, g, b)));
  }

  private void redLED(){
    setLED(255, 0, 0);
  }

  private void greenLED(){
    setLED(0,255,0);
  }

  private void blueLED()
  {
    setLED(0, 0, 255);
  }

  private void whiteLED(){
    setLED(255, 255, 255);
  }

  private void turnOff(){
    setLED(0, 0, 0);
  }

  private void goldLED(){
    setLED(217, 160, 15);
  }

  private void goldTwinkle(){
    twinkleLED(217, 160, 15);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void bindCommands() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'bindCommands'");
  }
}
