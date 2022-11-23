package frc.robot.io;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import frc.robot.Constants;
import frc.robot.Robot;

public class DriveTrainEncoders {

    private final WPI_TalonFX talonLF;
    private final WPI_TalonFX talonRF;

    private final DifferentialDrivetrainSim driveTrainSim;

    private double robotZeroPositionLeft;
    private double robotZeroPositionRight;

    public DriveTrainEncoders(WPI_TalonFX talonLF, WPI_TalonFX talonRF, DifferentialDrivetrainSim driveTrainSim) {
        this.talonLF = talonLF;
        this.talonRF = talonRF;
        this.driveTrainSim = driveTrainSim;
        this.robotZeroPositionLeft = 0;
        this.robotZeroPositionRight = 0;
    }

    public double getDistancePassedLeftM() {
        if (Robot.isReal()) {
            return talonLF.getSelectedSensorPosition() / Constants.TALON_FX_PPR / Constants.DRIVE_GEAR_RATIO * Constants.DRIVE_WHEEL_CIRCUMEFERENCE_M;
        } else {
            return driveTrainSim.getLeftPositionMeters() - robotZeroPositionLeft;
        }
    }

    public double getDistancePassedRightM() {
        if (Robot.isReal()) {
            return talonRF.getSelectedSensorPosition() / Constants.TALON_FX_PPR / Constants.DRIVE_GEAR_RATIO * Constants.DRIVE_WHEEL_CIRCUMEFERENCE_M;
        } else {
            return driveTrainSim.getRightPositionMeters() - robotZeroPositionRight;
        }
    }

    public double getLeftVelocityMetersPerSecond() {
        if (Robot.isReal()) {
            return talonLF.getSelectedSensorPosition() / Constants.TALON_FX_PPR / Constants.DRIVE_GEAR_RATIO * 10.0 * Constants.DRIVE_WHEEL_CIRCUMEFERENCE_M;
        } else {
            return driveTrainSim.getLeftVelocityMetersPerSecond();
        }
    }

    public double getRightVelocityMetersPerSecond() {
        if (Robot.isReal()) {
            return talonRF.getSelectedSensorVelocity() / Constants.TALON_FX_PPR / Constants.DRIVE_GEAR_RATIO / 10.0 * Constants.DRIVE_WHEEL_CIRCUMEFERENCE_M;
        } else {
            return driveTrainSim.getRightVelocityMetersPerSecond();
        }
    }

    public double getDistancePassedM() {
        return (getDistancePassedLeftM() + getDistancePassedRightM()) / 2.0;
    }

    public void resetEncoders() {
        if (Robot.isReal()) {
            talonLF.setSelectedSensorPosition(0);
            talonRF.setSelectedSensorPosition(0);
        } else {
            this.robotZeroPositionLeft = driveTrainSim.getLeftPositionMeters();
            this.robotZeroPositionRight = driveTrainSim.getRightPositionMeters();
        }
    }
}
