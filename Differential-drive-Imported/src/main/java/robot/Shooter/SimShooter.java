package robot.Shooter;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import robot.Constants;
import static robot.Shooter.ShooterConstants.*;

public class SimShooter implements ShooterIO {
    private final FlywheelSim shooter;

    public SimShooter() {

        shooter = new FlywheelSim(LinearSystemId.createFlywheelSystem(DCMotor.getNeoVortex(2), MOI, GEARING),
                DCMotor.getNeoVortex(2));

    }

    public void setVoltage(double voltage) {
        shooter.setInputVoltage(voltage);
        shooter.update(Constants.PERIOD.in(Seconds));
    }

    public double getVelocity() {
        return shooter.getAngularVelocityRadPerSec();
    }
}