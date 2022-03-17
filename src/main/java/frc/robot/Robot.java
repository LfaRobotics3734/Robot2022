// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.util.net.PortForwarder;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();
  
  private XboxController xbox_control;

  ShuffleboardTab mainBoard = Shuffleboard.getTab("Main");
  
  // DRIVETRAIN motors and corresponding variables
  private WPI_TalonSRX backLeft, backRight, frontLeft, frontRight;
  private final double drivetrainVoltageComp = 10;
  private double leftSpeed;
  private double rightSpeed;
  private double rotationSpeed;
  // private DifferentialDrive drive;


  // INTAKE motor and corresponding variables
  private PWMVictorSPX intakeRoller;
  private boolean intakeOn;

  // INTAKE ANGLE motor and corresponding
  private TalonSRX intakeAngle;

  // INDEXER Motors and corresponding variables
  private TalonSRX indexer;
  private double indexerSpeed;
  private boolean automaticIndexerOn;

  private AnalogInput sen1;
  private AnalogInput sen2;
  private AnalogInput sen3;
  private boolean sen1Active;
  private boolean sen2Active;
  private boolean sen3Active;
  private boolean toSen2;
  private boolean toSen3;

  private PIDController indexerPID;
  private SimpleMotorFeedforward indexerFeedForward;

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
  private targetGrouping cameraYawResult;
  private List<PhotonTrackedTarget> targets;
  private double currentYaw = 0;
  private int consecutiveCorrect = 0;
  
  PIDController rotationController;
  PIDController forwardController;
  //kP values for targeting and distance
  private double rotationKp;
  private double distanceKp;


  /* Constants for moving to specific distance */
  final double CAMERA_HEIGHT_METERS = Units.inchesToMeters(26.25);
  final double TARGET_HEIGHT_METERS = Units.inchesToMeters(96);//104 real
  // Angle between horizontal and the camera. Current one allows shots from 7 foot 6 inches to 14 feet
  final double CAMERA_PITCH_RADIANS = Units.degreesToRadians(30.0);// on the front wheels
  // How far from the target we want to be
  //0.6 is constant to add from distances on sheets and 2 feet is goal radius
  final double[] GOAL_RANGES_METERS = {Units.feetToMeters(7.5+0.6), Units.feetToMeters(9+0.6),Units.feetToMeters(15+0.6)};
  final double[] RPS = {33.5, 36, 35};
  final double[] HOOD_ANGLE_DEG = {35, 36, 38};

  private boolean pitchTargeting = false;


  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);

    //JOYSTICK in PORT 1, check FRC driver station
    xbox_control = new XboxController(1);

    /* Allows access to photonvision dashboard over USB/wifi,
     * go to 10.37.34.19:5800 when connected.
     * If that doesn't work, arp -a in terminal and try port
     * 5800 in browser for the ip adresses that show up */
    PortForwarder.add(5800, "photonvision.local", 5800);
    
    //DRIVETRAIN
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
    leftSpeed = 0;
    rightSpeed = 0;
    rotationSpeed = 0;
    //kp constants
    rotationKp = 0.39;
    distanceKp = 10.0; //Original was 7.0
  
    rotationController = new PIDController(rotationKp, 0, 0);
    forwardController=new PIDController(distanceKp, 0, 0);
    SmartDashboard.putNumber("rotationKp", rotationKp);
    SmartDashboard.putNumber("distanceKp", distanceKp);
    
    //INTAKE
    intakeAngle = new TalonSRX(5);
    intakeRoller = new PWMVictorSPX(5);
    intakeOn = false;
    
    //INDEXER
    indexer = new TalonSRX(4);
    indexer.configFactoryDefault();
    indexer.setInverted(true);
    indexer.enableVoltageCompensation(true);
    indexer.configVoltageCompSaturation(11);
    indexerSpeed = 0;
    automaticIndexerOn = false;
    sen1 = new AnalogInput(0);
    sen1.setOversampleBits(4);
    sen1.setAverageBits(8);
    sen2 = new AnalogInput(1);
    sen2.setOversampleBits(4);
    sen2.setAverageBits(10);
    sen3 = new AnalogInput(2);
    sen3.setOversampleBits(4);
    sen3.setAverageBits(10);
    sen1Active = false;
    sen2Active = false;
    sen3Active = false;
    
    //SHOOTER MOTORS (Channel) - initializing encoders and PIDs
    shooter1 = new CANSparkMax(1, MotorType.kBrushless);
    shooter2 = new CANSparkMax(2, MotorType.kBrushless);
    shooter1.restoreFactoryDefaults();
    shooter2.restoreFactoryDefaults();
    shooter2.follow(shooter1, true);
    shooterPID = new PIDController(2.7528E-09, 0, 0);
    shooterFeedForward = new SimpleMotorFeedforward(0.24231, 0.18758, 0.0052013);
    shooterOn = false;
    rpsSetpoint = 0;
    SmartDashboard.putNumber("rps", rpsSetpoint);

    //SHOOTER ANGLE - initializing shooter angle motor, encoders, and PIDs
    shooterAngle = new TalonSRX(3);
    shooterAngle.configFactoryDefault();
    shooterAngle.setInverted(false);
    shooterAngle.setSensorPhase(false);
    shooterAngle.enableVoltageCompensation(true);
    shooterAngle.configVoltageCompSaturation(shooterAngleVoltageComp);
    shooterAngle.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 0, 30);
    shooterAnglePID = new PIDController(0.1, 0, 0);
    shooterAngleSetpoint = 1;
    shooterAngle.setSelectedSensorPosition(50);
    SmartDashboard.putNumber("angle", shooterAngleSetpoint);
    
    //Vision
    camera = new PhotonCamera("mmal_service_16.1");

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
    // Function Buttons
    if (xbox_control.getLeftBumper() && xbox_control.getRightBumper()) {
      if(xbox_control.getStartButtonPressed()) {
        shooterOn = !shooterOn;
        if (shooterOn) {
          rpsSetpoint = 28;
        } else {
          rpsSetpoint = 0;
        }
      }

      if (xbox_control.getBButton()) {
        shooterAngleSetpoint = 40;
      } else {
        shooterAngleSetpoint = 1;
      }

      if (xbox_control.getXButtonPressed()) {
        automaticIndexerOn = !automaticIndexerOn;
      }
    } else { // Normal Functions
      if(xbox_control.getBackButtonPressed()) {
        camera.setDriverMode(!camera.getDriverMode());
        System.out.println("Driver Mode Toggled. " + camera.getDriverMode());
      }

      if(!xbox_control.getYButton()) {
         // MANUAL DRIVETRAIN CONTROLS
         if (xbox_control.getRightTriggerAxis() < 0.05) {
          leftSpeed = 0;
          rightSpeed = 0;
        } else {
          leftSpeed = rightSpeed = Math.pow(xbox_control.getRightTriggerAxis(), 2);
        }

        if (xbox_control.getRightTriggerAxis() < 0.05 && xbox_control.getLeftTriggerAxis() > 0.05) {
          leftSpeed = rightSpeed = -1 * Math.pow(xbox_control.getLeftTriggerAxis(), 2);
        }

        if (Math.abs(xbox_control.getRightX()) > 0.2) {
          rotationSpeed = Math.pow(xbox_control.getRightX(), 2);
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
        target();
       }

      // shut things off after autotarget and shot
      if(xbox_control.getYButtonReleased()) {
        rpsSetpoint = 0;
        shooterAngleSetpoint = 1;
        shooterOn = false;
        pitchTargeting = false;
        currentYaw = 0;
        // consecutiveCorrect = 0;
      }
      
      // INTAKE CONTROLS
      if (xbox_control.getStartButtonPressed()) {
        intakeOn = !intakeOn;
      }

      // INDEXER CONTROLS
      if (!automaticIndexerOn) {
        if (xbox_control.getXButton()) {
          indexerSpeed = 1.0;
        } else {
          indexerSpeed = 0.0;
        }
      } else {
        System.out.println(sen1Active + ", " + sen2Active + ", " + sen3Active + ", " + toSen2 + ", " + toSen3 + ", " + sen1.getAverageValue() + ", " + sen2.getAverageValue() + ", " + sen3.getAverageValue() + ", " + indexerSpeed);
        
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
              }
            }
          }
        }

        if (toSen2 || toSen3) {
          if (toSen2 && sen2Active) {
            toSen2 = false;
            indexerSpeed = 0.0;
          } else if (toSen3 && sen3Active) {
            toSen3 = false;
            indexerSpeed = 0.0;
          }
        } else {
          if (xbox_control.getXButton()) {
            indexerSpeed = 1.0;
          } else {
            indexerSpeed = 0.0;
          }
        }
      }

      // RESET HOOD CONTROLS
      shooterResetting = xbox_control.getRightStickButton();
      if(xbox_control.getRightStickButtonReleased()) {
        shooterAngle.setSelectedSensorPosition(50);
      }
    }

    // DRIVETRAIN OUTPUT
    frontLeft.set(ControlMode.PercentOutput, leftSpeed);
    frontRight.set(ControlMode.PercentOutput, rightSpeed);
    
    // INTAKE OUTPUT
    if (intakeOn) {
      intakeRoller.set(-1.0);
    } else {
      intakeRoller.set(0.0);
    }

    // INDEXER OUTPUT
    indexer.set(ControlMode.PercentOutput, indexerSpeed);

    // SHOOTER OUTPUT
    if (shooterOn) {
      shooter1.setVoltage(shooterFeedForward.calculate(rpsSetpoint) + shooterPID.calculate(shooter1.getEncoder().getVelocity() * 1.5 / 60, rpsSetpoint));
    } else {
      shooter1.setVoltage(0.0);
    }

    // SHOOTER ANGLE OUTPUT
    if(!shooterResetting) {
      double currentSensorPosition = toDeg(shooterAngle.getSelectedSensorPosition());
      double output = shooterAnglePID.calculate(currentSensorPosition, shooterAngleSetpoint) / shooterAngleVoltageComp;
      output = MathUtil.clamp(output, -1, 1);
      shooterAngle.set(ControlMode.PercentOutput, output);
    } else {
      shooterAngle.set(ControlMode.PercentOutput, -0.5);
    }
  }

  private void target() {
    result = camera.getLatestResult();
    
    if(result.hasTargets()) {
      //stop for camera to process
      cameraPitchResult = result.getBestTarget();
      targets = result.getTargets();
      
      // cameraYawResult = new targetGrouping(targets);

      leftSpeed = 0;
      rightSpeed = 0;

      int bestTargetingDistance = 0;

      double distanceToGoal =
        PhotonUtils.calculateDistanceToTargetMeters(
                CAMERA_HEIGHT_METERS,
                TARGET_HEIGHT_METERS,
                CAMERA_PITCH_RADIANS,
                Units.degreesToRadians(cameraPitchResult.getPitch()));
        

      if(!pitchTargeting) {
        rotationSpeed = rotationController.calculate(cameraPitchResult.getYaw(), 0) / drivetrainVoltageComp;
        rotationSpeed = MathUtil.clamp(rotationSpeed, -1.0, 1.0);
        leftSpeed = rotationSpeed;
        rightSpeed = -rotationSpeed;
        // targets = cameraPitchResult.getTargets();
        // target = cameraPitchResult.getBestTarget();
      } else {
        double currentLowestDistance = distanceToGoal;
        for(int i = 0; i < GOAL_RANGES_METERS.length; i++) {
          if(Math.abs(GOAL_RANGES_METERS[i] - distanceToGoal) < currentLowestDistance) {
            currentLowestDistance = Math.abs(GOAL_RANGES_METERS[i] - distanceToGoal);
            bestTargetingDistance = i;
          }
        }
        
        rpsSetpoint = RPS[bestTargetingDistance];
        shooterOn = true;
        shooterAngleSetpoint = HOOD_ANGLE_DEG[bestTargetingDistance];
        double forwardSpeed = -forwardController.calculate(distanceToGoal, GOAL_RANGES_METERS[bestTargetingDistance]) / drivetrainVoltageComp;
        forwardSpeed=MathUtil.clamp(forwardSpeed,-1.0, 1.0);
        System.out.println("Speed: " + forwardSpeed + " Distance: " + distanceToGoal + " Goal distance: " + GOAL_RANGES_METERS[bestTargetingDistance]);
        leftSpeed = forwardSpeed;
        rightSpeed = forwardSpeed;
        // targets = cameraYawResult.getTargets();
        // target = cameraYawResult.getBestTarget();
      }
      
      //System.out.println("Goal: " + Units.metersToFeet(GOAL_RANGES_METERS[bestTargetingDistance]) + ", Actual: " + Units.metersToFeet(distanceToGoal) + ", Angle: " + shooterAngleSetpoint + ", Speed: " + rpsSetpoint);
      //if rotationSpeed less than certain point, then set to 0

      // System.out.println("Yaw: " + cameraYawResult.getYaw() + ", Distance: " + Units.metersToFeet(distanceToGoal) + "Pitch Targeting: " + pitchTargeting);
      //drive.arcadeDrive(rotationSpeed, forwardSpeed);
      // if(cameraYawResult.getYaw() < 1) {
      //   consecutiveCorrect++;
      // } else {
      //   consecutiveCorrect = 0;
      // }

      if(!pitchTargeting && rotationSpeed < 0.1) {
        pitchTargeting = true;
      }
      // System.out.println("Pitch Targeting: " + pitchTargeting + ", Angle: " + cameraYawResult.getYaw());
    }
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
        break;
      case kDefaultAuto:
      default:
        // Put default auto code here
        break;
    }
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {}

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

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

  public double toDeg(double pos) {
    double deg = pos*360.0/4096.0;
    deg*=10;
    deg=(int) deg;
    deg/=10;
    return deg;
  }

  public double toRad(double deg) {
    return deg / 360 * 2 * Math.PI;
  }
  
  // Emulates built-in PhotonVision target grouping pipeline
  private class targetGrouping {
    private double yaw = 0;
    public targetGrouping(List<PhotonTrackedTarget> targetList) {
      for(var i = 0; i < targetList.size(); i++) {
        // if(targetList.get(i).getArea()>0.1){
        //   yaw += targetList.get(i).getYaw();
        // }
        yaw += targetList.get(i).getYaw();
      }
      yaw /= targetList.size();
      // if(currentYaw != 0) {
      //   yaw += currentYaw;
      //   yaw /= 2;
      //   currentYaw = yaw;
      // } else {
      //   currentYaw = yaw;
      // }
    }

    public double getYaw() {
      return yaw;
    }
  }
}

// rpsSetpoint = SmartDashboard.getNumber("rps", 0);
// shooterAngleSetpoint = (int)SmartDashboard.getNumber("angle", 0);

/* //testing kp values for pid
if(rotationKp!=SmartDashboard.getNumber("rotationKp", 0)){
  rotationKp=SmartDashboard.getNumber("rotationKp", 0);
  rotationController = new PIDController(rotationKp, 0, rotationKd);
}
if(distanceKp!=SmartDashboard.getNumber("distanceKp", 0)){
  distanceKp=SmartDashboard.getNumber("distanceKp", 0);
  forwardController=new PIDController(distanceKp, 0, 0);
}
//testing kd values for pid
if(rotationKd!=SmartDashboard.getNumber("rotationKd", 0)){
  rotationKd=SmartDashboard.getNumber("rotationKd", 0);
  rotationController = new PIDController(rotationKp, 0, rotationKd);
} */