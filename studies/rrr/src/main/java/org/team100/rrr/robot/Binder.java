package org.team100.rrr.robot;

import static org.team100.lib.util.TriggerUtil.onTrue;
import static org.team100.lib.util.TriggerUtil.whileTrue;

import org.team100.lib.commands.MoveAndHold;
import org.team100.lib.geometry.se2.VelocitySE2;
import org.team100.lib.hid.DriverXboxControl;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.profile.r1.ProfileR1;
import org.team100.lib.profile.r1.WPITrapezoidProfileR1;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Commands;

public class Binder {
    private final Machinery m_machinery;
    private final DriverXboxControl m_driver;

    public Binder(LoggerFactory rootLogger, Machinery machinery) {
        LoggerFactory log = rootLogger.type(this);
        m_machinery = machinery;
        m_driver = new DriverXboxControl(log, 0);

        // Default control is "joint" mode.
        // m_machinery.m_arm.setDefaultCommand(
        // m_machinery.m_arm.moveJointsManually(m_driver::velocity));
        m_machinery.m_arm.setDefaultCommand(
                m_machinery.m_arm.movePoseManually(m_driver::velocity));

        // Right bumper puts the control in "pose" mode.
        onTrue(m_driver::leftBumper, Commands.runOnce(
                () -> m_machinery.m_arm.setDefaultCommand(
                        m_machinery.m_arm.movePoseManually(m_driver::velocity))));
        // Left bumper puts it back in "joint" mode.
        onTrue(m_driver::rightBumper, Commands.runOnce(
                () -> m_machinery.m_arm.setDefaultCommand(
                        m_machinery.m_arm.moveJointsManually(m_driver::velocity))));

        // "Y" is button 4, "v" in sim
        ProfileR1 profile = new WPITrapezoidProfileR1(3, 6);

        // profiles in joint space make kinda circular paths in workspace
        MoveAndHold move1 = m_machinery.m_arm.moveProfiled(profile,
                new Pose2d(0.5, 0.25, new Rotation2d(0)));
        MoveAndHold move2 = m_machinery.m_arm.moveProfiled(profile,
                new Pose2d(0.6, 0.25, new Rotation2d(0)));
        MoveAndHold move3 = m_machinery.m_arm.moveProfiled(profile,
                new Pose2d(0.6, -0.25, new Rotation2d(0)));
        MoveAndHold move4 = m_machinery.m_arm.moveProfiled(profile,
                new Pose2d(0.5, -0.25, new Rotation2d(0)));
        whileTrue(m_driver::a,
                move1.until(move1::isDone)
                        .andThen(move2.until(move2::isDone))
                        .andThen(move3.until(move3::isDone))
                        .andThen(move4.until(move4::isDone)));

        // spline ends are kinda straighter
        // note approach directions.
        MoveAndHold move1s = m_machinery.m_arm.moveSplined(
                new VelocitySE2(0, 0.3, 0),
                new Pose2d(0.45, 0.25, new Rotation2d(0)),
                new VelocitySE2(0, 0.3, 0));
        MoveAndHold move2s = m_machinery.m_arm.moveSplined(
                new VelocitySE2(0.15, 0, 0),
                new Pose2d(0.6, 0.25, new Rotation2d(0)),
                new VelocitySE2(0.15, 0, 0));
        MoveAndHold move3s = m_machinery.m_arm.moveSplined(
                new VelocitySE2(0, -0.3, 0),
                new Pose2d(0.6, -0.25, new Rotation2d(0)),
                new VelocitySE2(0, -0.3, 0));
        MoveAndHold move4s = m_machinery.m_arm.moveSplined(
                new VelocitySE2(-0.15, 0, 0),
                new Pose2d(0.45, -0.25, new Rotation2d(0)),
                new VelocitySE2(-0.15, 0, 0));
        whileTrue(m_driver::b,
                move1s.until(move1s::isDone)
                        .andThen(move2s.until(move2s::isDone))
                        .andThen(move3s.until(move3s::isDone))
                        .andThen(move4s.until(move4s::isDone)));

        // trajectories in workspace make straight lines in workspace
        MoveAndHold move1t = m_machinery.m_arm.moveTrajSE2(log,
                new Pose2d(0.45, 0.25, new Rotation2d(0)), 1);
        MoveAndHold move2t = m_machinery.m_arm.moveTrajSE2(log,
                new Pose2d(0.6, 0.25, new Rotation2d(0)), 1);
        MoveAndHold move3t = m_machinery.m_arm.moveTrajSE2(log,
                new Pose2d(0.6, -0.25, new Rotation2d(0)), 1);
        MoveAndHold move4t = m_machinery.m_arm.moveTrajSE2(log,
                new Pose2d(0.45, -0.25, new Rotation2d(0)), 1);
        whileTrue(m_driver::x,
                move1t.until(move1t::isDone)
                        .andThen(move2t.until(move2t::isDone))
                        .andThen(move3t.until(move3t::isDone))
                        .andThen(move4t.until(move4t::isDone)));
    }

    public void close() {
    }
}
