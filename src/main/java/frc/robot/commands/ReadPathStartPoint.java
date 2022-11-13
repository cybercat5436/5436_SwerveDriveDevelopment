// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.io.File;
import java.io.IOException;
import java.nio.file.Path;
import java.util.Arrays;
import java.util.List;

import com.fasterxml.jackson.core.type.TypeReference;
import com.fasterxml.jackson.databind.ObjectMapper;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.cybercatclasses.PathFromJson;

public class ReadPathStartPoint extends CommandBase {
  /** Creates a new ReadPathStartPoint. */
  public ReadPathStartPoint() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    ObjectMapper objectMapper = new ObjectMapper();

    // File file = new File("deploy/Unnamed.wpilib.json");
    try {
      Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve("output/Unnamed.wpilib.json");
      File file = trajectoryPath.toFile();
      
      //
      // List<PathFromJson> pathFromJson = objectMapper.readValue(file, new TypeReference<List<PathFromJson>>(){} );
      List<PathFromJson> pathFromJson = Arrays.asList(objectMapper.readValue(file, PathFromJson[].class));  // 10x faster than TypeReference above
      //
      PathFromJson p = pathFromJson.get(0);
      System.out.println(p);
      System.out.println(p.toJson());
    } catch (IOException e) {
      // TODO Auto-generated catch block
      e.printStackTrace();
    }
    

    // Car car = objectMapper.readValue(file, Car.class);
      }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
