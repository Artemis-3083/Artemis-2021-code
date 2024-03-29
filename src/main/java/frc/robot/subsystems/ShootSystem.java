package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotCharacteristics;
import frc.robot.io.ShooterEncoder;
import org.apache.commons.math3.analysis.UnivariateFunction;
import org.apache.commons.math3.analysis.polynomials.PolynomialFunctionLagrangeForm;

import java.util.HashMap;
import java.util.Map;

public class ShootSystem extends SubsystemBase {

    private WPI_TalonFX shootRMotor;
    private WPI_TalonFX shootLMotor;
    public UnivariateFunction interpolation;

    private final FlywheelSim flywheelSim;
    private final PIDController pidControllerSim;
    private final ShooterEncoder encoder;

    public ShootSystem() {
        shootLMotor = new WPI_TalonFX(9);
        shootRMotor = new WPI_TalonFX(8);

        shootLMotor.setInverted(true);

        shootRMotor.set(TalonFXControlMode.Follower, 9);

        if (Robot.isSimulation()) {
            flywheelSim = new FlywheelSim(
                    RobotCharacteristics.SHOOTER_MOTORS,
                    RobotCharacteristics.SHOOTER_MOTOR_TO_WHEEL_GEAR_RATIO,
                    RobotCharacteristics.SHOOTER_MOMENT_OF_INERTIA
            );

            pidControllerSim = new PIDController(0.17, 0.0005, 0.0001, 0.02);
        } else {
            flywheelSim = null;
            pidControllerSim = null;

            /*shootLMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);
            shootLMotor.config_kP(0, 0.17);
            shootLMotor.config_kI(0, 0.0005);
            shootLMotor.config_kD(0, 0.0001);
            shootLMotor.config_kF(0, 0);

            shootLMotor.configPeakOutputForward(1);
            shootLMotor.configPeakOutputReverse(0);
            shootLMotor.configNominalOutputForward(0);
            shootLMotor.configNominalOutputReverse(0);*/
        }

        encoder = new ShooterEncoder(shootLMotor, flywheelSim);

        interpolation = createInterpolationFunction();

        NetworkTable pid = NetworkTableInstance.getDefault().getTable("pid");
        NetworkTableEntry kp = pid.getEntry("kp");
        kp.setDouble(0);
        NetworkTableEntry ki = pid.getEntry("ki");
        ki.setDouble(0);
        NetworkTableEntry kd = pid.getEntry("kd");
        kd.setDouble(0);
        NetworkTableEntry kf = pid.getEntry("kf");
        kf.setDouble(0);
        
        kp.addListener((notification)-> {
            shootLMotor.config_kP(0, notification.value.getDouble());
        }, EntryListenerFlags.kUpdate);
        ki.addListener((notification)-> {
            shootLMotor.config_kI(0, notification.value.getDouble());
        }, EntryListenerFlags.kUpdate);
        kd.addListener((notification)-> {
            shootLMotor.config_kD(0, notification.value.getDouble());
        }, EntryListenerFlags.kUpdate);
        kf.addListener((notification)-> {
            shootLMotor.config_kF(0, notification.value.getDouble());
        }, EntryListenerFlags.kUpdate);
    }

    public double getShooterRpm() {
        return encoder.getShooterRpm();
    }

    public void ShootAtRpm(double rpm){
        if (Robot.isReal()) {
            double velocity = rpm / 600 * Constants.TALON_FX_PPR / Constants.SHOOTER_GEAR_RATIO;
            shootLMotor.set(ControlMode.Velocity, velocity);
        } else {
            double output = pidControllerSim.calculate(getShooterRpm(), rpm);
            output = MathUtil.clamp(output, 0, 1);
            shootLMotor.set(output);
        }
    }

    public void Shoot(double speed){
        shootLMotor.set(speed);
        SmartDashboard.putNumber("Shooter Voltage", shootLMotor.getMotorOutputVoltage());
    }

    public void shootVoltage(double voltage) {
        shootLMotor.set(voltage / RobotController.getBatteryVoltage());
    }

    public void StopShoot(){
        shootLMotor.set(0);
        shootRMotor.set(0);
    }

    @Override
    public void simulationPeriodic() {
        flywheelSim.setInputVoltage(shootLMotor.get() * RobotController.getBatteryVoltage());
        flywheelSim.update(0.02);
    }

    private static UnivariateFunction createInterpolationFunction() {
        Map<Double, Double> dataPoints = getInterpolationDataPoints();
        double[] x = unboxArray(dataPoints.keySet().toArray(new Double[0]));
        double[] y = unboxArray(dataPoints.values().toArray(new Double[0]));
        return new PolynomialFunctionLagrangeForm(x, y);
    }
    
    private static Map<Double, Double> getInterpolationDataPoints() {
        Map<Double, Double> interpolationPoints = new HashMap<>();
        interpolationPoints.put(156.0, 2005.0);
        interpolationPoints.put(170.0, 2050.0);
        interpolationPoints.put(200.0, 2100.0);
        interpolationPoints.put(233.0, 2185.0);
        interpolationPoints.put(263.0, 2245.0);
        interpolationPoints.put(296.0, 2300.0);
        interpolationPoints.put(326.0, 2335.0);
        interpolationPoints.put(360.0, 2550.0);
        //interpolationPoints.put(distance in cm, speed in rpm);

        return interpolationPoints;
    }

    private static double[] unboxArray(Double[] arr) {
        double[] unboxed = new double[arr.length];
        for (int i = 0; i < arr.length; i++) {
            unboxed[i] = arr[i];
        }

        return unboxed;
    }
}