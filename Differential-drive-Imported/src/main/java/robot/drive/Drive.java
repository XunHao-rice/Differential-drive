package robot.drive;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import robot.Ports;


public class Drive extends SubsystemBase {
  private final SparkMax leftLeader = new SparkMax(Ports.Drive.LEFT_LEADER, MotorType.kBrushless);
  private final SparkMax leftFollower = new SparkMax(Ports.Drive.LEFT_FOLLOWER, MotorType.kBrushless);
  private final SparkMax rightLeader = new SparkMax(Ports.Drive.RIGHT_LEADER, MotorType.kBrushless);
  private final SparkMax rightFollower = new SparkMax(Ports.Drive.RIGHT_FOLLOWER, MotorType.kBrushless);

  public Drive() {
    SparkMaxConfig globalConfig = new SparkMaxConfig();
    SparkMaxConfig rightLeaderConfig = new SparkMaxConfig();
    SparkMaxConfig leftLeaderConfig = new SparkMaxConfig();
    SparkMaxConfig leftFollowerConfig = new SparkMaxConfig();
    SparkMaxConfig rightFollowerConfig = new SparkMaxConfig();

    globalConfig.idleMode(IdleMode.kBrake);

    leftFollowerConfig.apply(globalConfig).follow(Ports.Drive.LEFT_LEADER);
    rightFollowerConfig.apply(globalConfig).follow(Ports.Drive.RIGHT_LEADER);
    leftLeaderConfig.apply(globalConfig).inverted(true);
    rightLeaderConfig.apply(globalConfig).inverted(false);

    leftLeader.configure(leftLeaderConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    rightLeader.configure(rightLeaderConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    leftFollower.configure(leftFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    rightFollower.configure(rightFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  public Command drive(double leftSpeed, double rightSpeed) {
    return run(() -> {
      leftLeader.set(leftSpeed);
      rightLeader.set(rightSpeed);
    });
  }

}
