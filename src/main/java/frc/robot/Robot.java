// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.text.DecimalFormat;
import java.util.List;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonTargetSortMode;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.net.PortForwarder;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  /* JOYSTICKS */
  private XboxController xbox_control;
  private Joystick joystick;


  // DRIVETRAIN motors and corresponding variables
  private WPI_TalonSRX backLeft, backRight, frontLeft, frontRight;
  private final double drivetrainVoltageComp = 10;
  private double leftSpeed;
  private double rightSpeed;
  private double rotationSpeed;
  private double forwardSpeed;

  // INTAKE motor and corresponding variables
  private PWMVictorSPX intakeRoller;
  private boolean intakeOn;

  // INTAKE ANGLE motor and corresponding variables
  private TalonSRX intakeAngle;
  private double intakeAngleSpeed;

  // INDEXER Motors and corresponding variables
  private TalonSRX indexer;
  private double indexerSpeed;
  private boolean automaticIndexerOn;

  // INDEXER sensors
  private AnalogInput sen1;
  private AnalogInput sen2;
  private AnalogInput sen3;
  private boolean sen1Active;
  private boolean sen2Active;
  private boolean sen3Active;
  private boolean toSen2;
  private boolean toSen3;
  private Timer missedIntakeTimer;

  // SHOOTER motors and corresponding variables
  private CANSparkMax shooter1, shooter2;
  private PIDController shooterPID;
  private SimpleMotorFeedforward shooterFeedForward;
  private boolean shooterOn;
  private double rpsSetpoint;

  // SHOOTER ANGLE motor and corresponding variables
  private TalonSRX shooterAngle;
  private PIDController shooterAnglePID;
  private final int shooterAngleVoltageComp = 3;
  private double shooterAngleSetpoint;
  private boolean shooterResetting = false;

  // PHOTON CAMERA
  private PhotonCamera camera;
  private PhotonPipelineResult result;
  private PhotonTrackedTarget cameraPitchResult;
  //private targetGrouping cameraYawResult;
  private PhotonTrackedTarget cameraYawResult;
  private int consecutiveCorrect = 0;
  
  //PID Controllers
  PIDController rotationController;
  PIDController forwardController;

  // kP values for targeting and distance
  private double rotationKp;
  private double distanceKp;

  /* Constants for moving to specific distance */
  final double CAMERA_HEIGHT_METERS = Units.inchesToMeters(26.25);
  final double TARGET_HEIGHT_METERS = Units.inchesToMeters(104);// 104 real
  final double CAMERA_PITCH_RADIANS = Units.degreesToRadians(27.0); 
  private double distanceToGoal;
  // How far from the target we want to be
  // 0.6 is constant to add from distances on sheets and 2 feet is goal radius
  // final double GOAL_RANGES_METERS = Units.feetToMeters(11 + 0.6);
  // private double RPS = 38.5;//49;
  // private double HOOD_ANGLE_DEG = 33;

  // final double GOAL_RANGES_METERS = Units.feetToMeters(9 + 0.6);
  final double GOAL_RANGES_METERS = 3;
  private double RPS = 33.5;
  private double HOOD_ANGLE_DEG = 33.5;
  
  //CLOSE SHOT
  private double CLOSE_SHOT=25;

  //If already in right distance
  private boolean pitchTargeting = false;

  //FPV 
  UsbCamera fpvCamera;

  /* Autonomous Code */
  private Timer autonomousExtraIntakeTimer;
  private Timer autonomousWaitShootingTimer;
  private Timer autonomousIntakeAngleTimer;
  private Timer autonomousTimeoutMovingForward;
  private Timer autonomousFinishAuto;
  private boolean autonomousTargeting;
  private boolean foundTargets;
  private boolean alreadySetZero;
  private boolean autonomousStartShooting;
  private boolean autonomousInited;
  private double intakeSpeed;

  //Constant for emergency
  private double rpsConstant = 0;

  //Timer for Match Current Time
  private Timer matchTimer;

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  @Override
  public void robotInit() {
     
    // XBOX Controller in PORT 1, check FRC driver station
    xbox_control = new XboxController(1);
    joystick = new Joystick(0);

    // Set up FPV camera
    // CameraServer.startAutomaticCapture(

    /*
     * Allows access to photonvision dashboard over USB/wifi,
     * go to 10.37.34.19:5800 when connected.
     * If that doesn't work, arp -a in terminal and try port
     * 5800 in browser for the ip adresses that show up
     */
    
    PortForwarder.add(5800, "photonvision.local", 5800);

    // DRIVETRAIN
    backLeft = new WPI_TalonSRX(0);
    backRight = new WPI_TalonSRX(6);
    frontLeft = new WPI_TalonSRX(1);
    frontRight = new WPI_TalonSRX(2);

    backLeft.follow(frontLeft);
    backRight.follow(frontRight);
    backLeft.enableVoltageCompensation(true);
    backLeft.configVoltageCompSaturation(drivetrainVoltageComp);
    backRight.enableVoltageCompensation(true);
    backRight.configVoltageCompSaturation(drivetrainVoltageComp);
    frontLeft.enableVoltageCompensation(true);
    frontLeft.configVoltageCompSaturation(drivetrainVoltageComp);
    frontRight.enableVoltageCompensation(true);
    frontRight.configVoltageCompSaturation(drivetrainVoltageComp);
    backLeft.setInverted(true);
    frontLeft.setInverted(true);
    

    // kp constants
    rotationKp = 0.6; // Original 0.6
    distanceKp = 7.0; // Original was 7.0

    rotationController = new PIDController(rotationKp, 0, 0);
    forwardController = new PIDController(distanceKp, 0, 0);

    // INTAKE
    
    intakeAngle = new TalonSRX(5);
    intakeAngle.enableVoltageCompensation(true);
    intakeAngle.configVoltageCompSaturation(5);
    intakeRoller = new PWMVictorSPX(5);
    

    // INDEXER
    indexer = new TalonSRX(4);
    indexer.configFactoryDefault();
    indexer.setInverted(true);
    indexer.enableVoltageCompensation(true);
    indexer.configVoltageCompSaturation(11);
    
    sen1 = new AnalogInput(0);
    sen1.setOversampleBits(4);
    sen1.setAverageBits(8);
    sen2 = new AnalogInput(1);
    sen2.setOversampleBits(4);
    sen2.setAverageBits(10);
    sen3 = new AnalogInput(2);
    sen3.setOversampleBits(4);
    sen3.setAverageBits(10);
   
    missedIntakeTimer = new Timer();

    // SHOOTER MOTORS (Channel) - initializing encoders and PIDs
    shooter1 = new CANSparkMax(1, MotorType.kBrushless);
    shooter2 = new CANSparkMax(2, MotorType.kBrushless);
    shooter1.restoreFactoryDefaults();
    shooter2.restoreFactoryDefaults();
    shooter2.follow(shooter1, true);
    // shooter1.setSmartCurrentLimit(30, 90);
    // shooter2.setSmartCurrentLimit(30, 90);
    shooterPID = new PIDController(2.7528E-09, 0, 0);
    shooterFeedForward = new SimpleMotorFeedforward(0.24231, 0.18758, 0.0052013);
    //SHOOTER testing
    SmartDashboard.putNumber("RPS", RPS);
    SmartDashboard.putNumber("ANGLE", HOOD_ANGLE_DEG);
   /*  SmartDashboard.putNumber("rotationKp", rotationKp); */
    SmartDashboard.putNumber("CLOSE_SHOT", CLOSE_SHOT);

    // SHOOTER ANGLE - initializing shooter angle motor, encoders, and PIDs
    shooterAngle = new TalonSRX(3);
    shooterAngle.configFactoryDefault();
    shooterAngle.setInverted(false);
    shooterAngle.setSensorPhase(false);
    shooterAngle.enableVoltageCompensation(true);
    shooterAngle.configVoltageCompSaturation(shooterAngleVoltageComp);
    shooterAngle.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 0, 30);
    shooterAnglePID = new PIDController(0.1, 0, 0);


    shooterAngle.setSelectedSensorPosition(50);

    // Vision
    camera = new PhotonCamera("mmal_service_16.1");

    /* Autonomous */
    autonomousExtraIntakeTimer = new Timer();
    autonomousWaitShootingTimer = new Timer();
    autonomousIntakeAngleTimer = new Timer();
    autonomousTimeoutMovingForward = new Timer();
    autonomousFinishAuto = new Timer();
    // Constant


    //MATCH TIMER
    matchTimer=new Timer();
  }
  public void init(){
    leftSpeed = 0;
    rightSpeed = 0;
    rotationSpeed = 0;

    intakeAngleSpeed = 0;

    intakeOn = false;

    indexerSpeed = 0;
    automaticIndexerOn = true;

    sen1Active = false;
    sen2Active = false;
    sen3Active = false;

    shooterOn = false;
    rpsSetpoint = 0;

    shooterAngleSetpoint = 1;
  }

  /** This function is called periodically during operator control. */
  @Override
  public void robotPeriodic() {
    intakeAngle.set(ControlMode.PercentOutput, intakeAngleSpeed);

    rpsConstant = 0.75 * (1 - joystick.getRawAxis(3));

    if (sen1.getAverageValue() < 300) {
      sen1Active = true;
    } else {
      sen1Active = false;
    }

    if (sen2.getAverageValue() < 300) {
      sen2Active = true;
    } else {
      sen2Active = false;
    }

    if (sen3.getAverageValue() < 300) {
      sen3Active = true;
    } else {
      sen3Active = false;
    }
    // DRIVETRAIN OUTPUT
    frontLeft.set(ControlMode.PercentOutput, leftSpeed);
    frontRight.set(ControlMode.PercentOutput, rightSpeed);

    // SHOOTER ANGLE OUTPUT
    if (!shooterResetting) {
      double currentSensorPosition = toDeg(shooterAngle.getSelectedSensorPosition());
      double output = shooterAnglePID.calculate(currentSensorPosition, shooterAngleSetpoint) / shooterAngleVoltageComp;
      output = MathUtil.clamp(output, -1, 1);
      shooterAngle.set(ControlMode.PercentOutput, output);
    } else {
      shooterAngle.set(ControlMode.PercentOutput, -0.5);
    }

    // SHOOTER OUTPUT
    if (shooterOn) {
      shooter1.setVoltage(shooterFeedForward.calculate(rpsSetpoint + rpsConstant)
          + shooterPID.calculate(shooter1.getEncoder().getVelocity() * 1.5 / 60, rpsSetpoint));
    } else {
      shooter1.setVoltage(0.0);
    }
    // INDEXER OUTPUT
    indexer.set(ControlMode.PercentOutput, indexerSpeed);
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    init();
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for
   * items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and
   * test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and
   * SmartDashboard integrated updating.
   */

  @Override
  public void teleopPeriodic() {
    /*  if(rotationKp!=SmartDashboard.getNumber ("rotationKp", 0)){
      rotationKp=SmartDashboard.getNumber("rotationKp", 0);
      rotationController = new PIDController(rotationKp, 0, 0);
    } */

    RPS = SmartDashboard.getNumber("RPS", 0);
    HOOD_ANGLE_DEG=SmartDashboard.getNumber("ANGLE", 0);

    CLOSE_SHOT = SmartDashboard.getNumber("CLOSE_SHOT", 0);

    // Normal Functions
    if (!joystick.getRawButton(3)) {
      // MANUAL DRIVETRAIN CONTROLS
      if (xbox_control.getRightTriggerAxis() < 0.05) {
        leftSpeed = 0;
        rightSpeed = 0;
      } else {
        leftSpeed = rightSpeed = Math.pow(xbox_control.getRightTriggerAxis(), 3);
      }

      if (xbox_control.getRightTriggerAxis() < 0.05 && xbox_control.getLeftTriggerAxis() > 0.05) {
        // leftSpeed = rightSpeed = -1 * Math.pow(xbox_control.getLeftTriggerAxis(), 3);
        leftSpeed = rightSpeed = -1 * Math.pow(xbox_control.getLeftTriggerAxis(), 2);
      }

      if (Math.abs(xbox_control.getRightX()) > 0.2) {
        // Was power of two
        rotationSpeed = Math.abs(Math.pow(xbox_control.getRightX(), 2)) / 1.5;
        if (xbox_control.getRightX() < 0) {
          rotationSpeed = -1 * rotationSpeed;
        }
        leftSpeed -= rotationSpeed;
        rightSpeed += rotationSpeed;
      }

      leftSpeed = MathUtil.clamp(leftSpeed, -1, 1);
      rightSpeed = MathUtil.clamp(rightSpeed, -1, 1);

    } else {
      // AUTO DRIVETRAIN CONTROLS
      if (hasTargets()) {
        target();
      } else {
        rotationSpeed = 0;
      }

    }

    // shut things off after autotarget and shot
    if (joystick.getRawButtonReleased(3)) {
      rpsSetpoint = 0;
      shooterAngleSetpoint = 1;
      shooterOn = false;
      pitchTargeting = false;
      consecutiveCorrect = 0;
    }

    // All off button
    if (joystick.getPOV() == 180) {
      shooterOn = false;
      intakeOn = false;
      indexerSpeed = 0;
    }

    // Short Shot
    if (joystick.getRawButton(4)) {
      rpsSetpoint = CLOSE_SHOT+rpsConstant;
      shooterAngleSetpoint = 1;
      shooterOn = true;
    }
    if (joystick.getRawButtonReleased(4)) {
      rpsSetpoint = 0;
      shooterOn = false;
    }

    if (joystick.getRawButton(7)) {
      shooterAngleSetpoint = 40;
    } else {
      shooterAngleSetpoint = 1;
    }

    // Intake
    if (joystick.getRawButtonPressed(2)) {
      intakeOn = !intakeOn;
    }

    // INDEXER CONTROLS

    // System.out.println(sen1Active + ", " + sen2Active + ", " + sen3Active + ", "
    // + toSen2 + ", " + toSen3 + ", " + sen1.getAverageValue() + ", " +
    // sen2.getAverageValue() + ", " + sen3.getAverageValue() + ", " +
    // indexerSpeed);

    if (sen1Active) {
      if (!sen3Active) {
        if (sen2Active) {
          if (!toSen3) {
            toSen3 = true;
            indexerSpeed = 0.3;
          }
        } else {
          if (!toSen2) {
            toSen2 = true;
            indexerSpeed = 0.3;
            missedIntakeTimer.start();
          }
        }
      }
    }

    if (toSen2 || toSen3) {
      if ((toSen2 && sen2Active) || missedIntakeTimer.get() > 1.5) {
        toSen2 = false;
        indexerSpeed = 0.0;
        missedIntakeTimer.stop();
        missedIntakeTimer.reset();
      } else if (toSen3 && sen3Active) {
        toSen3 = false;
        indexerSpeed = 0.0;
      }
    }

    if (joystick.getRawButton(1)) {
      indexerSpeed = 0.4; // Was .5
      automaticIndexerOn = false;
    } else if (joystick.getRawButton(5)) {
      indexerSpeed = -0.4;
      automaticIndexerOn = false;
    }

    if (!joystick.getRawButton(1) && !joystick.getRawButton(5) && !automaticIndexerOn) {
      indexerSpeed = 0.0;
      automaticIndexerOn = true;
    }

    // RESET HOOD CONTROLS
    shooterResetting = joystick.getRawButton(11);
    if (joystick.getRawButtonReleased(11)) {
      shooterAngle.setSelectedSensorPosition(50);
    }

    // INTAKE OUTPUT
    if (intakeOn) {
      intakeRoller.set(-0.5);
    } else {
      intakeRoller.set(0.0);
    }

    

  }

  private void target() {
    // stop for camera to process
    cameraPitchResult = result.getBestTarget();
    cameraYawResult=result.getBestTarget();
    //cameraYawResult = new targetGrouping(result.getTargets());
    leftSpeed = 0;
    rightSpeed = 0;

   /*  distanceToGoal = PhotonUtils.calculateDistanceToTargetMeters(
        CAMERA_HEIGHT_METERS,
        TARGET_HEIGHT_METERS,
        CAMERA_PITCH_RADIANS,
        Units.degreesToRadians(cameraPitchResult.getPitch()));


    if (!pitchTargeting) { */


      rotationSpeed = absCompensator(rotationController.calculate(cameraYawResult.getYaw(), 0) / drivetrainVoltageComp);

      // System.out.println("Original: " + rotationController.calculate(cameraYawResult.getYaw(), 0) / drivetrainVoltageComp + ", Altered: " + rotationSpeed);
      rotationSpeed = MathUtil.clamp(rotationSpeed, -1.0, 1.0);
      leftSpeed = rotationSpeed;
      rightSpeed = -rotationSpeed;

      /* if (rotationSpeed < 0.2) { // was .1, changed for accuracy 4/7 4:39
        consecutiveCorrect++;
      } else {
        consecutiveCorrect = 0;

      }
      
    } else {
      rpsSetpoint = RPS;
      shooterOn = true;
      shooterAngleSetpoint = HOOD_ANGLE_DEG;

      forwardSpeed = -forwardController.calculate(distanceToGoal, GOAL_RANGES_METERS) / drivetrainVoltageComp;
      forwardSpeed = MathUtil.clamp(forwardSpeed, -1.0, 1.0);
      leftSpeed = forwardSpeed;
      rightSpeed = forwardSpeed;
    // System.out.println(
          // "Pitch Targeting: " + forwardSpeed + ", Distance: " + distanceToGoal + ", Yaw: " + cameraYawResult.getYaw());
    } */
    //// System.out.println("Goal: " +
    //// Units.metersToFeet(GOAL_RANGES_METERS[bestDistanceIndex]) + ", Actual: " +
    //// Units.metersToFeet(distanceToGoal) + ", Angle: " + shooterAngleSetpoint +
    //// ", Speed: " + rpsSetpoint);

    // // System.out.println("Yaw: " + cameraYawResult.getYaw() + ", Distance: " +
    // Units.metersToFeet(distanceToGoal) + "Pitch Targeting: " + pitchTargeting);

   /*  if (!pitchTargeting && consecutiveCorrect == 25) {
      pitchTargeting = true;
    } */
    // // System.out.println("Pitch Targeting: " + pitchTargeting + ", Angle: " +
    // cameraYawResult.getYaw());
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different
   * autonomous modes using the dashboard. The sendable chooser code works with
   * the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the
   * chooser code and
   * uncomment the getString line to get the auto name from the text box below the
   * Gyro
   *
   * <p>
   * You can add additional auto modes by adding additional comparisons to the
   * switch structure
   * below with additional strings. If using the SendableChooser make sure to add
   * them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    /* Starting Match Timer */
    matchTimer.reset();
    matchTimer.start();
    init();
    // make sure ball aligned on camera 
    // go forward until sense another ball
    // intake, turn until shooter and shoot
    
    // USE ACTUAL BALL DISTANCES
    // PUT FIRST BALL FURTHER IN
    /* reset booleans */
    autonomousTargeting = false;
    alreadySetZero=false;
    foundTargets=false; 
    autonomousInited=false;
    autonomousStartShooting=false;
    /* reset timers */
    autonomousExtraIntakeTimer.reset();
    autonomousWaitShootingTimer.reset();
    autonomousIntakeAngleTimer.reset();
    autonomousTimeoutMovingForward.reset();
    autonomousFinishAuto.reset();
    /* Initialize */
    autonomousIntakeAngleTimer.start();
    autonomousTimeoutMovingForward.start();
    intakeAngleSpeed = -0.5;
    intakeSpeed=0.0;
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    /* rpsSetpoint = CLOSE_SHOT;
    shooterAngleSetpoint = 1;
    shooterOn = true;
    indexerSpeed = 0.4;
    autonomousFinishAuto.start();
    if(autonomousFinishAuto.get() > 4) {
      shooterOn = false;
      indexerSpeed = 0;
      init();
    }
   */
  
    double currentTime = autonomousTimeoutMovingForward.get();
    if(currentTime<=1){
        leftSpeed=currentTime*0.5;
        rightSpeed=currentTime*0.5;
    }
    if (autonomousIntakeAngleTimer.get() > 0.5) {
      intakeAngleSpeed = 0.0;
      intakeSpeed=-0.5;
    }
    if(autonomousStartShooting==true&&(!sen1Active && !sen2Active && !sen3Active)) {
      autonomousFinishAuto.start();
      intakeSpeed = 0.0;
    }
    if(autonomousFinishAuto.get() > 3&&!autonomousInited) {
      autonomousInited=true;
      shooterOn = false;
      init();
      
    }

    // if (autonomousExtraIntakeTimer.get() > 3.5) {
    //   intakeSpeed=0.0;
    //   autonomousExtraIntakeTimer.stop();
    // }
    intakeRoller.set(intakeSpeed);
    if (autonomousTargeting) {
      if (!foundTargets) {
        leftSpeed=0.33;
        rightSpeed=-0.33;
      }
      // target if detect goal
      result = camera.getLatestResult();
      if (hasTargets()) {
        if (!alreadySetZero) {
          alreadySetZero = true;
          leftSpeed=0;
          rightSpeed=0;
        }
        foundTargets = true;
        target();
        if (rpsSetpoint > 0) {
          autonomousWaitShootingTimer.start();
        }
        if (autonomousWaitShootingTimer.get() > 3) {
          indexerSpeed=0.4;
          autonomousStartShooting=true;
        }
    
      }

    } else if (sen1Active || autonomousTimeoutMovingForward.get() > 4) {
      leftSpeed=0;
      rightSpeed=0;
      autonomousTargeting = true;
      // autonomousExtraIntakeTimer.start();
    }

  }

  private boolean hasTargets() {
    result = camera.getLatestResult();
    try{
      String log = "Current Time: "+new DecimalFormat("#.##").format(matchTimer.get())+", Rotation Speed: " + new DecimalFormat("#.##").format(rotationSpeed) +", Distance To Goal (Meters): "+new DecimalFormat("#.##").format(distanceToGoal)+", Forward Speed: "+ new DecimalFormat("#.##").format(forwardSpeed)+", Pitch Targeting: "+pitchTargeting +", Consecutive Correct: "+consecutiveCorrect;

      if(cameraYawResult != null) {
        log += ", Yaw: "+new DecimalFormat("#.##").format(cameraYawResult.getYaw());
      }

      if(cameraPitchResult!=null){
        log+=", Pitch: "+cameraPitchResult.getPitch();
        // new DecimalFormat("#.##").format()
      }
      try {
        System.out.println("Camera Has Targets: "+result.hasTargets() + ", " + log);
      } catch(Exception e) {
        System.out.println("Error: " + e.getCause().toString() + ", " + log);
      } 
    } catch (Exception e){
      System.out.println("Failure");
    }

    if (result.hasTargets()) {
      //LOG: result targets amount
      return true;
      /* if (result.getTargets().size() >= 2) {
        return true;
      } */
    }
    return false;
  }

  private double absCompensator(double input) {
    double output = (-0.84 * Math.abs(input)) + 1.5;
    output *= input;
    return output;
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {
  }

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {
  }

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
  }

  public double toDeg(double pos) {
    double deg = pos * 360.0 / 4096.0;
    deg *= 10;
    deg = (int) deg;
    deg /= 10;
    return deg;
  }

  public double toRad(double deg) {
    return deg / 360 * 2 * Math.PI;
  }

  // Emulates built-in PhotonVision target grouping pipeline
  private class targetGrouping {
    private double yaw = 0;

    public targetGrouping(List<PhotonTrackedTarget> targetList) {
      double avgArea = 0;
      yaw = 0;
      for (var i = 0; i < targetList.size(); i++) {
        avgArea += targetList.get(i).getArea();
      }
      avgArea /= targetList.size();
      for (var i = 0; i < targetList.size(); i++) {
        // yaw += targetList.get(i).getYaw() * targetList.get(i).getArea() / totalArea;
        if(targetList.get(i).getArea() > avgArea / 4) {
          yaw += targetList.get(i).getYaw();
        }
      }
      //CHANGE TO DIVIDE TO GOOD TARGETS ONLY
      yaw /= targetList.size();
    }

    public double getYaw() {
      return yaw;
    }
  }
}

