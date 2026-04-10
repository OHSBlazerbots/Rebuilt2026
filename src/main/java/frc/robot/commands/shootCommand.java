package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class shootCommand extends Command {

    private final ShooterSubsystem m_Shooter;
    private final FeederSubsystem m_Feeder;
    private final SwerveSubsystem m_Swerve;
    private final int shooterspeed;
    private final Timer timer = new Timer();

    public shootCommand(ShooterSubsystem shooter, FeederSubsystem feeder, SwerveSubsystem swerve, int shooterVelocity) {
        m_Shooter = shooter;
        m_Feeder = feeder;
        m_Swerve = swerve;
        shooterspeed = shooterVelocity;


        addRequirements(m_Shooter, m_Feeder, m_Swerve);
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();

        m_Shooter.setShooterVelocity(shooterspeed);
        m_Shooter.startAngleMaker();
        System.out.println("Shoot Command Initialized");
    }

    @Override
    public void execute() {

        if (m_Shooter.isAtSetpoint(6500)) {
            m_Shooter.startKicker();
            m_Feeder.startRoller();
        }
        System.out.println("Shoot command executed");
    }

    @Override
    public void end(boolean interrupted) {
        m_Shooter.stopShooter();
        m_Shooter.stopKicker();
        m_Feeder.stopRoller();

        timer.stop();
        System.out.println("Shooting done");
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}