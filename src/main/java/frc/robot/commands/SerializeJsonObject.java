package frc.robot.commands;

import javax.sound.sampled.SourceDataLine;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.cybercatclasses.AbsEncoderOffsets;

public class SerializeJsonObject extends CommandBase{

    AbsEncoderOffsets absEncoderOffsets;

    public SerializeJsonObject(AbsEncoderOffsets absEncoderOffsets){
        this.absEncoderOffsets = absEncoderOffsets;
    }

    @Override
    public void end(boolean interrupted) {
        // TODO Auto-generated method stub
        super.end(interrupted);
    }

    @Override
    public void execute() {
        // TODO Auto-generated method stub
        System.out.println(AbsEncoderOffsets.getJsonEncoding(absEncoderOffsets));
    }

    @Override
    public void initialize() {
        // TODO Auto-generated method stub
        super.initialize();
    }

    @Override
    public boolean isFinished() {
        // TODO Auto-generated method stub
        return true;
    }

    
}
