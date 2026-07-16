package robot.Shooter;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;

public class ShooterConstants {

    public static final double GEARING = 1;
    public static final double RADIUS = 1;
    public static final Current CURRENT_LIMIT = Amps.of(40);
    public static final AngularVelocity DEFAULT_VELOCITY = RadiansPerSecond.of(4);
    public static final AngularVelocity MAX_VELOCITY = RadiansPerSecond.of(8);
    public static final double MAX_VOLTAGE = 12;
    public static final double MOI = 0;// moment of inertia;
    public static final double K_P = 1;
    public static final double K_I = 0;
    public static final double K_D = .1;
    public static final double K_S = 0;
    public static final double K_V = 0;
    public static final double K_A = 0;
}
