package frc.robot.subsystems.aki_testing;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.LoggedTunableNumber;

public class Flywheel extends SubsystemBase{
    private static final LoggedTunableNumber kP = 
        new LoggedTunableNumber("flywheel/kP", 0.0);

    private double velocity = 0.0;
    private double targerVelocity = 100.0;

    @Override 
    public void periodic() {
        double currentkP = kP.get();

        double error = targerVelocity - velocity;
        double output = currentkP * error;

        velocity += output * 0.02;

        Logger.recordOutput("flywheel/Velocity:", velocity);
        Logger.recordOutput("Flywheel/Error", error);
    }
}
