package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.Constants.IntakeConstants;




public class intakeCommand extends Command {
    private final IntakeSubsystem m_Intake;

    public intakeCommand(IntakeSubsystem intake) {
        m_Intake = intake;
    

        addRequirements(m_Intake);
    }

    @Override
    public void initialize() {
        m_Intake.pivotOut();


    }

    @Override
    public void execute() {

        if (m_Intake.isDown()){
            m_Intake.rollersIn();
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_Intake.stopRollers();
        m_Intake.pivotIn();
        System.out.println("Intaking balls done");
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}