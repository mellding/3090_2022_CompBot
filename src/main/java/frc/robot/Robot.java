
package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import com.revrobotics.CANSparkMax;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoSink;

public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  //Analog Inputs
  private static final int hiPressurePort = 0;
  private static final int loPressurePort = 1;
  AnalogInput hiPressureIN = new AnalogInput(hiPressurePort);
  AnalogInput loPressureIN = new AnalogInput(loPressurePort);
  double hiPressure;
  double loPressure;
  

  //CAN Bus Addresses
  private static final int left0DevID = 11;   private static final int left1DevID = 12;
  private static final int right0DevID = 13;  private static final int right1devID = 14;
  private static final int liftMotor0ID = 15; private static final int pigIMUDevID = 16;

  CANSparkMax leftMotor0 = new CANSparkMax(left0DevID, MotorType.kBrushless);
  CANSparkMax leftMotor1 = new CANSparkMax(left1DevID, MotorType.kBrushless);
  CANSparkMax rightMotor0= new CANSparkMax(right0DevID, MotorType.kBrushless);
  CANSparkMax rightMotor1= new CANSparkMax(right1devID,MotorType.kBrushless);

  TalonSRX liftMotor = new TalonSRX(liftMotor0ID);
  
    
  private SparkMaxPIDController pidControllerLeft;
  private SparkMaxPIDController pidControllerRight;
  private RelativeEncoder leftEncoder;
  private RelativeEncoder rightEncoder;

  private static final int driveMaxApmps = 35;
  private static final double rampRate = 1;

  double rightTrigger = 0;
  double leftTrigger = 0;
  double airPressure = 0;
  double throttle = 0;
  double turn = 0;
  double maxLoSpeed = .375;
  double soldeoidOnTime = 200;

  boolean launcherForward = true;

  XboxController control00 = new XboxController(0);

  Compressor c = new Compressor(1, PneumaticsModuleType.CTREPCM);
  Solenoid solenoid0 = new Solenoid(PneumaticsModuleType.CTREPCM, 0);
  Solenoid solenoid1 = new Solenoid(PneumaticsModuleType.CTREPCM, 1);

  DifferentialDrive botDrive = new DifferentialDrive(leftMotor0,rightMotor0);

  UsbCamera cameraLancher;
  UsbCamera cameraPickup;
  VideoSink server;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser); 
    
    SmartDashboard.putNumber("MaxDriveAmps", driveMaxApmps); 
    
    SmartDashboard.putNumber("Solenoid Time", soldeoidOnTime);


    leftMotor0.restoreFactoryDefaults(); leftMotor1.restoreFactoryDefaults();
    rightMotor0.restoreFactoryDefaults(); rightMotor1.restoreFactoryDefaults();
 

    leftMotor0.setSmartCurrentLimit(driveMaxApmps); leftMotor1.setSmartCurrentLimit(driveMaxApmps);
    rightMotor0.setSmartCurrentLimit(driveMaxApmps); rightMotor1.setSmartCurrentLimit(driveMaxApmps);

    leftMotor0.setIdleMode(IdleMode.kCoast); leftMotor1.setIdleMode(IdleMode.kCoast);
    rightMotor0.setIdleMode(IdleMode.kCoast); rightMotor1.setIdleMode(IdleMode.kCoast);

    rightMotor0.setInverted(true);rightMotor1.setInverted(true);

    liftMotor.setNeutralMode(NeutralMode.Brake);

    hiPressureIN.setAverageBits(6);
    loPressureIN.setAverageBits(6);

    c.enabled();
    solenoid0.set(false);
    solenoid1.set(false);

    cameraLancher = CameraServer.startAutomaticCapture(0);
    cameraPickup = CameraServer.startAutomaticCapture(1);
    server = CameraServer.getServer();

    leftMotor0.burnFlash();leftMotor1.burnFlash();
    rightMotor0.burnFlash();rightMotor1.burnFlash();

  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {

    leftEncoder = leftMotor0.getEncoder();
    rightEncoder = rightMotor0.getEncoder();

    hiPressure = 250 * hiPressureIN.getAverageVoltage() / 5 - 25;
    loPressure = 250 * loPressureIN.getAverageVoltage() / 5 - 25;


    SmartDashboard.putNumber("throttle = ", throttle);
    SmartDashboard.putNumber("turn = ", turn);
    SmartDashboard.putNumber("Left Amps", leftMotor0.getOutputCurrent());
    SmartDashboard.putNumber("Right Amps", rightMotor0.getOutputCurrent());
    SmartDashboard.putNumber("Right Enc= ", rightEncoder.getPosition());
    SmartDashboard.putNumber("Left Enc=", leftEncoder.getPosition());
    SmartDashboard.putNumber("HiPres = ", hiPressure);
    SmartDashboard.putNumber("LoPres", loPressure);
    SmartDashboard.getNumber("MaxLoSpeed", maxLoSpeed);
    SmartDashboard.getNumber("Solenoid Time", soldeoidOnTime);
    SmartDashboard.putBoolean("LaunchFront", launcherForward);
    //SmartDashboard.putBoolean("A Button", control00.getAButton());

  }

  /**
   * This autonomous (along with the chooser code above) shows how to select between different
   * autonomous modes using the dashboard. The sendable chooser code works with the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
   * uncomment the getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to the switch structure
   * below with additional strings. If using the SendableChooser make sure to add them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);

  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
      case kCustomAuto:
        // Put custom auto code here
        // new ParallelDeadlineGroup(new WaitCommand(3), new RunCommand(() -> botDrive.arcadeDrive(0.375, 0)));
        // new ParallelDeadlineGroup(new WaitCommand(1), new RunCommand(() -> liftMotor.set(ControlMode.PercentOutput, -0.75)));
        // solenoid0.set(true);
        // solenoid1.set(true);
        // double startTime = System.currentTimeMillis();
        // while(System.currentTimeMillis() - startTime <= soldeoidOnTime){}
        // solenoid0.set(false);
        // solenoid1.set(false);

        break;
      case kDefaultAuto:
        // Put default auto code here
        //new ParallelDeadlineGroup(new WaitCommand(3), new RunCommand(() -> botDrive.arcadeDrive(0.375, 0)));
        //new ParallelDeadlineGroup(new WaitCommand(1), new RunCommand(() -> liftMotor.set(ControlMode.PercentOutput, -0.75)));
        
        double timerThing = System.currentTimeMillis();
        while(System.currentTimeMillis() - timerThing <= 5000){
          botDrive.arcadeDrive(0.375, 0);
        }
      
        timerThing = System.currentTimeMillis();
        while(System.currentTimeMillis() - timerThing <= 2000){
          botDrive.arcadeDrive(0, 0);
          liftMotor.set(ControlMode.PercentOutput, -.5);
        }

        timerThing = System.currentTimeMillis();
        while(System.currentTimeMillis() - timerThing <= 500) {
          liftMotor.set(ControlMode.PercentOutput, 0);
        }

        solenoid0.set(true);
        solenoid1.set(true);
        double startTime = System.currentTimeMillis();
        while(System.currentTimeMillis() - startTime <= soldeoidOnTime){}
        solenoid0.set(false);
        solenoid1.set(false);
        m_autoSelected = "done";
        break;
    }
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    //leftMotor0.

  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {

    
    maxLoSpeed =  SmartDashboard.getNumber("MaxLoSpeed", maxLoSpeed);
    soldeoidOnTime = SmartDashboard.getNumber("Solenoid Time", soldeoidOnTime); 


    //**********Drive Train Section*****************************
    if(control00.getBackButton()  && control00.getBackButtonReleased()){
      launcherForward = !launcherForward;
      server.getSource();
    }

    if(!launcherForward){
      if(control00.getRightTriggerAxis() > 0){
          throttle = control00.getRightTriggerAxis();
      }else if(control00.getLeftTriggerAxis() > 0){
          throttle  = control00.getLeftTriggerAxis() * -1;
      }else throttle = 0;
    }
    else{
      if(control00.getRightTriggerAxis() > 0){
        throttle = control00.getRightTriggerAxis() * -1;
      }else if(control00.getLeftTriggerAxis() > 0){
        throttle  = control00.getLeftTriggerAxis();
      }else throttle = 0;
    }
  
    turn  = control00.getLeftX();

    if(control00.getLeftBumper()){
      throttle  = throttle * maxLoSpeed;
      turn      = turn * maxLoSpeed;
    }

    botDrive.arcadeDrive(throttle, turn);
    //************End DRive Train Section ***********************


    /**********Launch Section */
      if(control00.getAButton() && control00.getAButtonReleased() && control00.getLeftBumper()){
        solenoid0.set(true);
        solenoid1.set(true);
        double startTime = System.currentTimeMillis();
        while(System.currentTimeMillis() - startTime <= soldeoidOnTime){}
        solenoid0.set(false);
        solenoid1.set(false);
      }else{
        solenoid0.set(false);
        solenoid1.set(false);
      }
    //********End Launch Section */


    if(Math.abs(control00.getRightY()) > .2){
        liftMotor.set(ControlMode.PercentOutput, -control00.getRightY());
    }else {
      liftMotor.set(ControlMode.PercentOutput, 0);
    }


  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}


}
