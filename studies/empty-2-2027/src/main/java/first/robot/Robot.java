// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package first.robot;

import org.wpilib.command2.Command;
import org.wpilib.command2.CommandScheduler;
import org.wpilib.framework.TimedRobot;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.TalonFX;
import com.reduxrobotics.canand.CanandEventLoop;
import com.reduxrobotics.sensors.canandmag.Canandmag;

public class Robot extends TimedRobot {
  private Command autonomousCommand;

  private final RobotContainer robotContainer;
  private final TalonFX f1 = new TalonFX(1, CANBus.systemcore(0));
  private final TalonFX f2 = new TalonFX(2, CANBus.systemcore(0));
  private final TalonFX f3 = new TalonFX(3, CANBus.systemcore(0));
  private final TalonFX f4 = new TalonFX(4, CANBus.systemcore(0));
  private final TalonFX f5 = new TalonFX(5, CANBus.systemcore(0));
  private final TalonFX f6 = new TalonFX(6, CANBus.systemcore(0));
  private final TalonFX f7 = new TalonFX(7, CANBus.systemcore(0));
  private final TalonFX f8 = new TalonFX(8, CANBus.systemcore(0));
  private final Canandmag m1 = new Canandmag(1);
  private final Canandmag m2 = new Canandmag(1);
  private final Canandmag m3 = new Canandmag(1);
  private final Canandmag m4 = new Canandmag(1);

  public Robot() {
    robotContainer = new RobotContainer();
    CanandEventLoop.getInstance();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    System.out.printf("[%.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f] [%.2f %.2f %.2f %.2f]\n",
        f1.getPosition().getValueAsDouble(),
        f2.getPosition().getValueAsDouble(),
        f3.getPosition().getValueAsDouble(),
        f4.getPosition().getValueAsDouble(),
        f5.getPosition().getValueAsDouble(),
        f6.getPosition().getValueAsDouble(),
        f7.getPosition().getValueAsDouble(),
        f8.getPosition().getValueAsDouble(),
        m1.getPosition(),
        m2.getPosition(),
        m3.getPosition(),
        m4.getPosition());
  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void disabledExit() {
  }

  @Override
  public void autonomousInit() {
    autonomousCommand = robotContainer.getAutonomousCommand();

    if (autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(autonomousCommand);
    }
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void autonomousExit() {
  }

  @Override
  public void teleopInit() {
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {
    f1.setThrottle(0.1);
    f2.setThrottle(0.1);
    f3.setThrottle(0.1);
    f4.setThrottle(0.1);
    f5.setThrottle(0.1);
    f6.setThrottle(0.1);
    f7.setThrottle(0.1);
    f8.setThrottle(0.1);
  }

  @Override
  public void teleopExit() {
  }

  @Override
  public void utilityInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void utilityPeriodic() {
  }

  @Override
  public void utilityExit() {
  }
}
