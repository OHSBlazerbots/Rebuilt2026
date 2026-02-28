import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.Constants;
import java.io.File;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.simulation.DoubleSolenoidSim;
import edu.wpi.first.wpilibj.simulation.PWMSim;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

//test imcomplete
class ShooterTest {
    ShooterSubsystem m_ShooterSubsystem;

    @BeforeEach // this method will run before each test
    void setup() {
        assert HAL.initialize(500, 0); // initialize the HAL, crash if failed
        m_ShooterSubsystem = new ShooterSubsystem();
    }

    @Test
    void shooterTest() {
        m_ShooterSubsystem.setColumnVelocity(67);
        m_ShooterSubsystem.setShooterVelocity(67);
        m_ShooterSubsystem.setLinearServoPosition(67);
    }

}