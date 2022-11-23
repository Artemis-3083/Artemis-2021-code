package frc.robot;

import java.util.List;

import org.ejml.equation.ManagerFunctions.Input1;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.commands.AutomaticShootingCommand;
import frc.robot.commands.CollectAndMoveDown;
import frc.robot.commands.CollectorCollectCommand;
import frc.robot.commands.CollectorPneumaticLiftCommand;
import frc.robot.commands.CollectorPneumaticLowerCommand;
import frc.robot.commands.ConveyorCommand;
import frc.robot.commands.DriveDistance;
import frc.robot.commands.FeedCommand;
import frc.robot.commands.LimeLightTowerAngle;
import frc.robot.commands.MoveElevatorDown;
import frc.robot.commands.MoveElevatorUp;
import frc.robot.commands.NewAutomaticShootingCommand;
import frc.robot.commands.NewDriveCommand;
import frc.robot.commands.ShootAtRpm;
import frc.robot.commands.ShootWithWheelsCommand;
import frc.robot.commands.ShooterTowerMoveCommand;
import frc.robot.commands.TogglePressure;
import frc.robot.commands.UnLoadCommand;
import frc.robot.commands.benDriveCommand;
import frc.robot.subsystems.CollectorSystem;
import frc.robot.subsystems.DriveSystem;
import frc.robot.subsystems.ElevatorSystem;
import frc.robot.subsystems.FeederSystem;
import frc.robot.subsystems.LimeLightImageProcessing;
import frc.robot.subsystems.ShootSystem;
import frc.robot.subsystems.TowerSystem;

public class Robot extends TimedRobot {

    private DriveSystem driveSystem;
    private CollectorSystem collectorsystem;
    private ShootSystem shootSystem;
    private TowerSystem towerSystem;
    private FeederSystem feederSystem;
    private LimeLightImageProcessing limeLightImageProcessing;
    private ElevatorSystem elevatorSystem;

    private PS4Controller controller;
    private PS4Controller seccontroller;

    private SendableChooser<Command> autoChooser;
    private Command autoCommand = null;
    NetworkTableEntry setpoint;
    NetworkTableEntry feed;
    Compressor compressor;
    
    @Override
    public void robotInit() {
        compressor = new Compressor(PneumaticsModuleType.CTREPCM);
        compressor.enableDigital();

        elevatorSystem = new ElevatorSystem();
        limeLightImageProcessing = new LimeLightImageProcessing();
        driveSystem = new DriveSystem();
        collectorsystem = new CollectorSystem();
        shootSystem = new ShootSystem();
        towerSystem = new TowerSystem();
        feederSystem = new FeederSystem();

        controller = new PS4Controller(0);
        seccontroller = new PS4Controller(2);

        CameraServer.startAutomaticCapture(0);

        /*
         *  OPERATOR CONTROL
         *
         *  CONTROLLER 1: DRIVER
         *      RIGHT STICK Y: DRIVE RIGHT
         *      LEFT STICK Y: DRIVE LEFT
         *
         *  CONTROLLER 2: SYSTEM OPERATOR
         *      TOWER
         *          RIGHT STICK X: TOWER ORIENTATION
         *          RIGHT STICK Y: TOWER ANGLE
         *
         *      COLLECTOR
         *          POV UP [HOLD]: COLLECTOR UP
         *          POV DOWN [HOLD]: COLLECTOR DOWN
         *          L1 [HOLD]: COLLECTOR IN
         *
         * 
         *      SHOOTER
         *          R2 [HOLD] (POWER): SHOOT
         *
         *      FEEDER
         *          ACTIVATED WHEN SHOOTING OR COLLECTING
         *          NOTE: CANNOT COLLECT AND SHOOT AT THE SAME TIME.
         */
        //shootSystem.setDefaultCommand(new ShootWithWheelsCommand(shootSystem, shootSystem.getShooterRpm()));

        //driveSystem.setDefaultCommand(new NewDriveCommand(driveSystem, controller));
        driveSystem.setDefaultCommand(new benDriveCommand(driveSystem, controller));
        towerSystem.setDefaultCommand(new ShooterTowerMoveCommand(towerSystem, seccontroller));
        //shootSystem.setDefaultCommand(new NewAutomaticShootingCommand(limeLightImageProcessing, shootSystem));
        //shootSystem.setDefaultCommand(new ShootWithWheelsCommand (shootSystem, shootSystem.getShooterRpm()));
        
        new POVButton(seccontroller, 180)
                .whileHeld(new CollectorPneumaticLiftCommand(collectorsystem));
        new POVButton(seccontroller, 0)
                .whileHeld(new CollectorPneumaticLowerCommand(collectorsystem));
        new JoystickButton(seccontroller, PS4Controller.Button.kL1.value)
                .whileHeld(new CollectorCollectCommand(collectorsystem, feederSystem));
        new JoystickButton(seccontroller, PS4Controller.Button.kCross.value)
                .whileHeld(new UnLoadCommand(feederSystem, collectorsystem));

        /*new JoystickButton(seccontroller, PS4Controller.Button.kR2.value)
                .whenActive(new ShootWithWheelsCommand(shootSystem, seccontroller, shootSystem.getShooterRpm()));*/
        new JoystickButton(seccontroller, PS4Controller.Button.kR2.value)
                .whileHeld(new NewAutomaticShootingCommand(limeLightImageProcessing, shootSystem));
        /*new JoystickButton(seccontroller, PS4Controller.Button.kR2.value)
                .whileHeld(new ShootAtRpm(shootSystem, 5000));*/

        new JoystickButton(seccontroller, PS4Controller.Button.kTriangle.value)
                .whenActive(new LimeLightTowerAngle(limeLightImageProcessing, towerSystem));
        new JoystickButton(seccontroller, PS4Controller.Button.kR1.value)
                .whileHeld(new FeedCommand(feederSystem));
        
        new JoystickButton(seccontroller, PS4Controller.Button.kCircle.value)
                .whileHeld(new ShootAtRpm(shootSystem, 800)); //215cm = 8.388 Voltage*/

        /*new JoystickButton(seccontroller, PS4Controller.Button.kR2.value)
                .whileHeld(new AutomaticShootingCommand(shootSystem, limeLightImageProcessing));*/


        new JoystickButton(seccontroller, PS4Controller.Button.kSquare.value)
                .whenActive(new TogglePressure(elevatorSystem));
        new POVButton(seccontroller, 270)
                .whileHeld(new MoveElevatorUp(elevatorSystem));
        new POVButton(seccontroller, 90)
                .whileHeld(new MoveElevatorDown(elevatorSystem));

        SmartDashboard.putData("Reset Tower Pos", new InstantCommand(()-> towerSystem.resetTowerPosition()));
        SmartDashboard.putData("Reset Driver Pos", new InstantCommand(()-> driveSystem.resetEncoders()));

        autoChooser = new SendableChooser<>();
        autoChooser.addOption("Do Nothing", new InstantCommand());
        autoChooser.addOption("Drive Passed Line", new DriveDistance(driveSystem, 2.5));
        SmartDashboard.putData("Auto", autoChooser);


        setpoint = NetworkTableInstance.getDefault().getTable("pid").getEntry("setpoint");
        setpoint.setDouble(0);
        feed = NetworkTableInstance.getDefault().getTable("pid").getEntry("feed");
        feed.setBoolean(false);
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        double interrpm = 0.0001*Math.pow(limeLightImageProcessing.getTargetDistance(), 3) - 0.0808 * Math.pow(limeLightImageProcessing.getTargetDistance(), 2) + 21.267 * limeLightImageProcessing.getTargetDistance() + 238.02;

        SmartDashboard.putNumber("Drive L", driveSystem.getDistancePassedLeftM());
        SmartDashboard.putNumber("Drive R", driveSystem.getDistancePassedRightM());
        SmartDashboard.putNumber("Drive motor LF", driveSystem.getDriveLfRpm());
        SmartDashboard.putNumber("Drive motor RF", driveSystem.getDriveRfRpm());
        SmartDashboard.putNumber("Drive motor LR", driveSystem.getDriveLrRpm());
        SmartDashboard.putNumber("Drive motor RR", driveSystem.getDriveRrRpm());
        SmartDashboard.putNumber("i shoot niggers for fun", interrpm);

        SmartDashboard.putNumber("Shooter Pos", towerSystem.getTowerPosition());
        SmartDashboard.putNumber("Shooter RPM", shootSystem.getShooterRpm());

        SmartDashboard.putBoolean("Tower Angle At Bottom", towerSystem.IsAngleAtBottom());
        SmartDashboard.putBoolean("Has Ball", feederSystem.HasBall()); //beware
        SmartDashboard.putBoolean("'is zeroed'", towerSystem.IsZeroed());

        SmartDashboard.putNumber("Vision Distance", limeLightImageProcessing.getTargetDistance());
        SmartDashboard.putNumber("Vision Horizontal Offset", limeLightImageProcessing.TxOffset());
        SmartDashboard.putNumber("interpolation rpm", NewAutomaticShootingCommand.rpm);
        SmartDashboard.putNumber("interpolation rpm", interrpm);

        if(towerSystem.IsZeroed()){
            towerSystem.resetTowerPosition();
        }
    }

    @Override
    public void disabledInit() {

    }

    @Override
    public void disabledPeriodic() {

    }

    @Override
    public void teleopInit() {

    }

    @Override
    public void teleopPeriodic() {

    }

    @Override
    public void autonomousInit() {
        //Command driveDistance = new DriveDistance(driveSystem, 3);
        //Command collectorDown = new CollectAndMoveDown(collectorsystem, feederSystem);
        /*autoCommand = new DriveDistance(driveSystem, 2).andThen(new WaitCommand(1), createShootCommand())
                .andThen(driveDistance.raceWith(collectorDown)).andThen(createShootCommand());*/
                //autoCommand = createShootCommand();
        //autoCommand = new DriveDistance(driveSystem, 1).andThen(new WaitCommand(1), createShootCommand());
        /*autoCommand = createShootCommand().andThen(driveDistance);
        if (autoCommand != null) {
                autoCommand.schedule();
        }*/
        DifferentialDriveVoltageConstraint autoVoltageConstraint =
                new DifferentialDriveVoltageConstraint(
                new SimpleMotorFeedforward(
                        RobotCharacteristics.ksVolts,
                        RobotCharacteristics.kvVoltSecondsPerMeter,
                        RobotCharacteristics.kaVoltSecondsSquaredPerMeter),
                        RobotCharacteristics.kDriveKinematics,
                10);

        // Create config for trajectory
        TrajectoryConfig config =
                new TrajectoryConfig(
                        RobotCharacteristics.kMaxSpeedMetersPerSecond,
                        RobotCharacteristics.kMaxAccelerationMetersPerSecondSquared)
                // Add kinematics to ensure max speed is actually obeyed
                .setKinematics(RobotCharacteristics.kDriveKinematics)
                // Apply the voltage constraint
                .addConstraint(autoVoltageConstraint);

        // An example trajectory to follow.  All units in meters.
        Trajectory exampleTrajectory =
                TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X direction
                new Pose2d(0, 0, new Rotation2d(0)),
                // Pass through these two interior waypoints, making an 's' curve path
                List.of(),
                // End 3 meters straight ahead of where we started, facing forward
                new Pose2d(1, 0, new Rotation2d(0)),
                // Pass config
                config);
        autoCommand = driveSystem.followTrajectory(exampleTrajectory);
        autoCommand.schedule();
    }

    private Command createShootCommand() {
        Command shoot = new NewAutomaticShootingCommand (limeLightImageProcessing, shootSystem);
        Command LimeligtAngle = new LimeLightTowerAngle(limeLightImageProcessing, towerSystem);
        Command waitFeed = new WaitCommand(2);
        Command feed = new FeedCommand(feederSystem);
        //Command beforeAuto = shoot.alongWith(LimeligtAngle);
        //return LimeligtAngle.andThen(shoot).alongWith(waitFeed.andThen(feed)).withTimeout(4);
        return new ShootAtRpm(shootSystem, 800).alongWith(new WaitCommand(2).andThen(feed)).withTimeout(3);
    }
    
    @Override
    public void autonomousPeriodic() {}

    @Override
    public void autonomousExit() {
        if (autoCommand != null) {
                autoCommand.cancel();
        }
    }

    @Override
    public void testInit() {

    }

    @Override
    public void testPeriodic() {
        new ShootSystem().ShootAtRpm(800);
    }
}
