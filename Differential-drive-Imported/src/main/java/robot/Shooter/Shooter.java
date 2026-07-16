package robot.Shooter;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import robot.Constants;
import robot.Robot;

public final class Shooter extends SubsystemBase {
    private final ShooterIO hardware;

    private final PIDController pid = new PIDController(ShooterConstants.K_P, ShooterConstants.K_I, ShooterConstants.K_D);
    private final SimpleMotorFeedforward ff = new SimpleMotorFeedforward(ShooterConstants.K_S, ShooterConstants.K_V,ShooterConstants. K_A, Constants.PERIOD.in(Seconds));

    private Shooter(ShooterIO hardware) {

        this.hardware = hardware;

    }

    public static Shooter create() {
        return Robot.isReal() ? new Shooter(new RealShooter()) : new Shooter(new SimShooter());
    }

    public static Shooter none() {
        return new Shooter(new NoShooter());
    }

    public double getVelocity() {
        return hardware.getVelocity();
    }

    public void update(double velocitySetpoint) {

        // makes sure the velocity it tries to go to does not exceed the max/min
        double velocity = MathUtil.clamp(velocitySetpoint,-ShooterConstants.MAX_VELOCITY.in(RadiansPerSecond), ShooterConstants.MAX_VELOCITY.in(RadiansPerSecond));

        // calculated the needed voltages using PID and FF
        double ffVolts = ff.calculate(velocity);
        double pidVolts = pid.calculate(getVelocity(), velocity);

        hardware.setVoltage(MathUtil.clamp(pidVolts + ffVolts, -ShooterConstants.MAX_VOLTAGE, ShooterConstants.MAX_VOLTAGE));

    }public Command runShooter(DoubleSupplier velocity) {
 return run(() -> update(velocity.getAsDouble()));
 }

}
