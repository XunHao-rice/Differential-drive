package robot.Shooter;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import static robot.Ports.Shooter.*;
import static robot.Shooter.ShooterConstants.*;



public class RealShooter implements ShooterIO {

   private final SparkMax leader;
   private final SparkMax follower;
   private final RelativeEncoder encoder;

   public RealShooter() {
       leader = new SparkMax(LEADER, MotorType.kBrushless);
       follower = new SparkMax(FOLLOWER, MotorType.kBrushless);
       encoder = leader.getEncoder();
SparkMaxConfig config = new SparkMaxConfig();
 config
       .smartCurrentLimit(40)
       .idleMode(IdleMode.kBrake);
        leader.configure(config, SparkBase.ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    follower.configure(config.inverted(true).follow(leader), SparkBase.ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
 leader.configure(config, SparkBase.ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    follower.configure(config.inverted(true).follow(leader), SparkBase.ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

}

@Override
public void setVoltage(double voltage) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'setVoltage'");
}

@Override
public double getVelocity() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'getVelocity'");
}

}