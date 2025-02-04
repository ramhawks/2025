// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkLimitSwitch;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANDigitalInput;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.hal.CANAPITypes.CANDeviceType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.Feeder;

public class feederSubsystem extends SubsystemBase {

  private final CANSparkMax outerMotor;
  private final CANSparkMax innerMotor;
  private final CANSparkMax positionMotor;

  private final RelativeEncoder outerMotorEncoder;
  private final RelativeEncoder innerMotorEncoder;
  private final RelativeEncoder positionEncoder;

  private final SparkPIDController positionMotorPID;
  private final SparkPIDController outerMotorPID;
  private final SparkPIDController innerMotorPID;

  DigitalInput intakeSwitch = new DigitalInput(Feeder.intakeSwitchPort);
  SparkLimitSwitch positionLimitSwitch;
  //Servo dropServo01 = new Servo(Feeder.dropServo01Port);
  //Servo dropServo02 = new Servo(Feeder.dropServo02Port);

  //public final ColorSensorV3 proxSensor;
  //private final I2C.Port i2cPort = I2C.Port.kOnboard;

  //private double proxDistance;

  //private final static lightingSubsystem lighting = new lightingSubsystem();

  private boolean isHomed = false;


  /** Creates a new feederSubsystem. */
  public feederSubsystem() {

    ///****************outer motor stuff***************/
    outerMotor = new CANSparkMax(Feeder.outerMotorID, MotorType.kBrushless);
    outerMotor.restoreFactoryDefaults();
    outerMotor.setSmartCurrentLimit(Feeder.outerMotorCurrentLimit); 
    outerMotor.setIdleMode(IdleMode.kBrake);
    outerMotor.setClosedLoopRampRate(.25);

    outerMotorEncoder = outerMotor.getEncoder();
    outerMotorPID = outerMotor.getPIDController();
    outerMotorPID.setP(Feeder.outerMotorKP);
    outerMotorPID.setI(Feeder.outerMotorKI);
    outerMotorPID.setD(Feeder.outerMotorKD);
    outerMotorPID.setOutputRange(Feeder.outerMotorMIN,Feeder.outerMotorMAX);
    ///****************end outer motor stuff***************/

    ///****************inner motor stuff***************/
    innerMotor = new CANSparkMax(Feeder.innerMotorID, MotorType.kBrushless);
    innerMotor.restoreFactoryDefaults();
    innerMotor.setSmartCurrentLimit(Feeder.innerMotorCurrentLimit);
    innerMotor.setIdleMode(IdleMode.kBrake);

    innerMotorEncoder = innerMotor.getEncoder();
    innerMotorPID = innerMotor.getPIDController();
    innerMotorPID.setP(Feeder.innerMotorKP);
    innerMotorPID.setI(Feeder.innerMotorKI);
    innerMotorPID.setD(Feeder.innerMotorKD);
    innerMotorPID.setOutputRange(Feeder.innerMotorMIN,Feeder.innerMotorMAX);
    ///****************end inner motor stuff***************/


    ///****************position motor stuff***************/
    positionMotor = new CANSparkMax(Feeder.positionMotorID, MotorType.kBrushless);
    positionMotor.restoreFactoryDefaults();
    positionMotor.setSmartCurrentLimit(Feeder.positionMotorCurrentLimit);
    positionMotor.setIdleMode(IdleMode.kBrake);

    positionLimitSwitch = positionMotor.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);

    positionEncoder = positionMotor.getEncoder();
    positionMotorPID= positionMotor.getPIDController();
    positionMotorPID.setP(Feeder.positionMotorKP);
    positionMotorPID.setI(Feeder.positionMotorKI);
    positionMotorPID.setD(Feeder.positionMotorKD);
    ///****************end position motor stuff***************/

    //lighting.setRGB(128, 52, 235);

    //dropServo01.set(0);
    //dropServo02.set(180);
    
    //proxSensor = new ColorSensorV3(i2cPort);


    outerMotor.burnFlash();
    innerMotor.burnFlash();
    positionMotor.burnFlash();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    //proxDistance = proxSensor.getProximity();

    //SmartDashboard.putNumber("rpm", outerMotorEncoder.getVelocity());
    //SmartDashboard.putNumber("outerAmps", outerMotor.getOutputCurrent());
    //SmartDashboard.putNumber("pos Encoder", positionEncoder.getPosition());
    //SmartDashboard.putBoolean("stop but2", RobotContainer.flightStick.getRawButton(11));
    SmartDashboard.putBoolean("Homed", isHomed);

    //if(!intakeSwitch.get()) stopMotors();
  }

  public void homeFeeder(){
    positionMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, false);
    positionMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, false);

    positionMotor.set(.5);
    while(!positionLimitSwitch.isPressed());
    positionMotor.set(0);
    positionEncoder.setPosition(0);

    positionMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, 0);
    positionMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, -200);
    positionMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
    positionMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);

    isHomed = true;

  }

  public void shootPosition(){
    if(!isHomed) return;
    positionMotorPID.setReference(-110, CANSparkMax.ControlType.kPosition);
  }

  public void intakePosition(){
    if(!isHomed) return;
    //gets the feeder into position for the intake to take in the "notes"
    positionMotorPID.setReference(-180, CANSparkMax.ControlType.kPosition);
    //while(Math.abs(positionEncoder.getPosition() + -170) > 5);
    
  }

  public void ampPosition(){
    if(!isHomed) return;
    //gets the feeder into position for the intake to deposit the "notes" in the amp
    positionMotorPID.setReference(0, CANSparkMax.ControlType.kPosition);
    //while(Math.abs(positionEncoder.getPosition() - 0) > 5);

    //wearIn();
  }

  public void travelPosition(){
    //sets the feeder into a low enough position to travel underneath the stage

  }

  public void intake(){
    if(!isHomed) return;
    //intake takes in the "notes"
    //intakePosition();

    //dropServo01.set(0);
    //dropServo02.set(180);

    //lighting.setRGB(235, 119, 52);

     if(!intakeSwitch.get()) {
    
      stopMotors();
    }else{
      outerMotorPID.setReference(Feeder.intakeSpeedOuter,  CANSparkMax.ControlType.kVelocity);
      innerMotorPID.setReference(Feeder.intakeSpeedInner, CANSparkMax.ControlType.kVelocity);
    }


   

    
  }

  public void stopMotors(){
    //outerMotorPID.setReference(0, CANSparkMax.ControlType.kVelocity);
    //innerMotorPID.setReference(0, CANSparkMax.ControlType.kVelocity);

    outerMotor.set(0);
    innerMotor.set(0);

    //lighting.setRGB(128, 52, 235);


    
    //dropServo01.set(0);
    //dropServo02.set(180);

   // innerMotorPID.setReference(innerMotorEncoder.getPosition(), CANSparkMax.ControlType.kPosition);
   // outerMotorPID.setReference(outerMotorEncoder.getPosition(), CANSparkMax.ControlType.kPosition);
  }

  public void depositAmp(){
    if(!isHomed) return;
    //deposits the "notes" into the amp 
    //ampPosition();

    

    //dropServo01.set(180);
    //dropServo02.set(0);

    //outerMotorPID.setReference(Feeder.depositSpeed,  CANSparkMax.ControlType.kVelocity);
    innerMotorPID.setReference(Feeder.depositSpeed, CANSparkMax.ControlType.kVelocity);
  }

  public void shootSpeaker(){
    if(!isHomed) return;
    //shoots speaker
    outerMotorPID.setReference(Feeder.shootSpeedOuter,  CANSparkMax.ControlType.kVelocity);

    while(outerMotorEncoder.getVelocity() < Feeder.shootSpeedOuter);

    //innerMotorPID.setReference(Feeder.shootSpeedInner, CANSparkMax.ControlType.kVelocity);
    innerMotor.set(1);
  }


  public void manaualMove(double input){
    if(Math.abs(input) > .1){
      positionMotor.set(input);
    }else positionMotor.set(0);
    
  }

  public void wearIn(){
    for(int i = 0; i < 10; i++){

        positionMotorPID.setReference(0, CANSparkMax.ControlType.kPosition);
        while(Math.abs(positionEncoder.getPosition() - 0) > 5);
        positionMotorPID.setReference(-200, CANSparkMax.ControlType.kPosition);
        while(Math.abs(positionEncoder.getPosition() + 200) > 5);
    }

    
  }

}
