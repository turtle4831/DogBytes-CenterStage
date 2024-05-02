package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.Components.Arm;

import kotlin.ParameterName;

public class ArmToPos extends CommandBase {
    Arm m_subsystem;
    public ArmToPos(Arm subsystem){
            this.m_subsystem = subsystem;
            addRequirements(this.m_subsystem);
    }

    @Override
    public void initialize() {
        m_subsystem.moveArmToPose(1);
    }

    @Override
    public void execute() {
        super.execute();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
