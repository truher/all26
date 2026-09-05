package org.team100.lib.subsystems.rrr;

import java.util.List;

import org.team100.lib.config.CurrentLimit;
import org.team100.lib.config.Friction;
import org.team100.lib.config.Identity;
import org.team100.lib.config.PIDConstants;
import org.team100.lib.dynamics.rrr.RRRDynamicsNewtonEuler;
import org.team100.lib.dynamics.rrr.RRREffort;
import org.team100.lib.framework.TimedRobot100;
import org.team100.lib.geometry.rrr.RRRAcceleration;
import org.team100.lib.geometry.rrr.RRRConfig;
import org.team100.lib.geometry.rrr.RRRState;
import org.team100.lib.geometry.rrr.RRRVelocity;
import org.team100.lib.geometry.se2.AccelerationSE2;
import org.team100.lib.geometry.se2.VelocitySE2;
import org.team100.lib.kinematics.rrr_se2.RRRFeasibility;
import org.team100.lib.kinematics.rrr_se2.RRRKinematicsPoE;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.TotalCurrentLog;
import org.team100.lib.mechanism.RotaryMechanism;
import org.team100.lib.motor.Motor;
import org.team100.lib.motor.MotorPhase;
import org.team100.lib.motor.NeutralMode100;
import org.team100.lib.motor.ctre.Falcon500Motor;
import org.team100.lib.motor.rev.Neo550CANSparkMotor;
import org.team100.lib.motor.sim.SimulatedMotor;
import org.team100.lib.state.ControlR1;
import org.team100.lib.state.ControlSE2;
import org.team100.lib.state.StateR1;
import org.team100.lib.state.StateSE2;
import org.team100.lib.util.CanId;
import org.team100.lib.util.StrUtil;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * A planar RRR arm where q2 is driven using a chain on the q1 axis, with equal
 * sized sprockets.
 * 
 * This arrangement couples q1 and q2 together.
 */
public class RRRArmCouple12 extends SubsystemBase implements RRRArm {
    private static final double DT = TimedRobot100.LOOP_PERIOD_S;
    private final LoggerFactory m_log;
    private final TotalCurrentLog m_currentLog;
    final RRRKinematicsPoE m_kinematics;
    final RRRDynamicsNewtonEuler m_dynamics;
    final RRRFeasibility m_feasibility;

    // q1 is driven with a gear bolted to the arm
    private final RotaryMechanism m_q1;
    // q2 is driven with a chain, driven from the q1 axis
    private final RotaryMechanism m_q2;
    // q3 has a flying motor
    private final RotaryMechanism m_q3;

    private RRRConfig m_q;
    private RRRVelocity m_qdot = new RRRVelocity(0, 0, 0);

    public RRRArmCouple12(LoggerFactory parent, TotalCurrentLog currentLog) {
        m_log = parent.type(this);
        m_currentLog = currentLog;
        LoggerFactory q1 = m_log.name("q1");
        LoggerFactory q2 = m_log.name("q2");
        LoggerFactory q3 = m_log.name("q3");
        // LINK LENGTHS, METERS
        double l1 = 0.3;
        double l2 = 0.3;
        double l3 = 0.1;
        m_kinematics = new RRRKinematicsPoE(l1, l2, l3);
        m_dynamics = RRRDynamicsNewtonEuler.thinRod(
                VecBuilder.fill(0, 0, 0),
                0.1, 0.1, 0.1,
                l1, l2, l3);
        RRRConfig qMin = new RRRConfig(-Math.PI / 2, -Math.PI / 2, -Math.PI / 2);
        RRRConfig qMax = new RRRConfig(Math.PI / 2, Math.PI / 2, Math.PI / 2);
        m_feasibility = new RRRFeasibility(m_kinematics, qMin, qMax);
        final Motor m1;
        final Motor m2;
        final Motor m3;
        if (Identity.instance.equals(Identity.TEST_BOARD_B0)
                || Identity.instance.equals(Identity.TEAM100_2018)) {
            m1 = new Falcon500Motor(
                    q1, m_currentLog, new CanId(5),
                    NeutralMode100.COAST, MotorPhase.FORWARD,
                    new CurrentLimit(20, 20), new Friction(0, 0, 0, 0),
                    PIDConstants.makePositionPID(1));
            m2 = new Falcon500Motor(
                    q2, m_currentLog, new CanId(21),
                    NeutralMode100.COAST, MotorPhase.FORWARD,
                    new CurrentLimit(20, 20), new Friction(0, 0, 0, 0),
                    PIDConstants.makePositionPID(1));
            m3 = new Neo550CANSparkMotor(
                    q3, m_currentLog, new CanId(14),
                    NeutralMode100.COAST, MotorPhase.FORWARD,
                    new CurrentLimit(20, 20), new Friction(0, 0, 0, 0),
                    PIDConstants.makePositionPID(1), 0, 0);
        } else {
            m1 = new SimulatedMotor(q1, 600);
            m2 = new SimulatedMotor(q2, 600);
            m3 = new SimulatedMotor(q3, 600);
        }
        // GEAR RATIOS
        double r1 = 7;
        double r2 = -5;
        double r3 = -12;

        m_q1 = new RotaryMechanism(
                q1, m1, m1.encoder(), 0, r1, qMin.q1(), qMax.q1());
        m_q2 = new RotaryMechanism(
                q2, m2, m2.encoder(), 0, r2, qMin.q2(), qMax.q2());
        m_q3 = new RotaryMechanism(
                q3, m3, m3.encoder(), 0, r3, qMin.q3(), qMax.q3());

        m_q = getConfig();
    }

    @Override
    public double l1() {
        return m_kinematics.l1;
    }

    @Override
    public double l2() {
        return m_kinematics.l2;
    }

    @Override
    public double l3() {
        return m_kinematics.l3;
    }

    @Override
    public void periodic() {
        m_q1.periodic();
        m_q2.periodic();
        m_q3.periodic();
    }

    @Override
    public RRRState set(RRRConfig q, RRRVelocity qdot, RRRAcceleration qddot) {
        RRREffort f = m_dynamics.effort(q, qdot, qddot);
        return set(q, qdot, f);
    }

    private RRRState set(RRRConfig q, RRRVelocity qdot, RRREffort f) {
        // q1 mechanism angle is the kinematic angle
        // q1 mechanism velocity is the kinematic velocity
        // q1 effort is the difference of kinematic efforts ... i think?
        // TODO: verify the effort coupling.
        m_q1.setUnwrappedPosition(q.q1(), qdot.q1dot(), f.t1() - f.t2());
        // q2 mechanism angle is the sum of the kinematic angles
        // q2 mechanism velocity is the sum of the kinematic velocities
        // q2 effort is just the kinematic effort ... I think?
        m_q2.setUnwrappedPosition(q.q2() + q.q1(), qdot.q2dot() + qdot.q1dot(), f.t2());
        // q3 is not coupled to anything.
        m_q3.setUnwrappedPosition(q.q3(), qdot.q3dot(), f.t3());

        q = getConfigWithinLimits();
        RRRAcceleration qddot = RRRAcceleration.solve(m_q, q, m_qdot, DT);
        // this accel will be very high when the arm "flips"
        // TODO: handle flipping
        System.out.printf("RRRArmCouple12: m_q [%s] q [%s] m_qdot [%s] qddot [%s]\n",
                m_q, q, m_qdot, qddot);
        RRRVelocity qdot2 = RRRVelocity.evolve(m_qdot, qddot, DT);
        m_q = q;
        m_qdot = qdot;
        if (qdot.toVector().norm() > 0.001 && qdot2.toVector().norm() > 2 * qdot.toVector().norm()) {
            m_qdot = new RRRVelocity(0, 0, 0);
        }
        return new RRRState(q, qdot);
    }

    /**
     * Choose the feasible config closest to the current config.
     * 
     * @param p tool center point pose
     */
    @Override
    public RRRConfig config(Pose2d p) {
        RRRConfig q0 = getConfig();
        System.out.printf("RRRArmCouple12: config %s\n", q0);
        List<RRRConfig> qAll = m_kinematics.inverse(p, q0.q1());
        if (qAll.isEmpty()) {
            System.out.println("RRRArmCouple12: no solution " + StrUtil.poseStr(p));
            return q0;
        }
        List<RRRConfig> qFeasible = m_feasibility.filter(qAll);
        if (qFeasible.isEmpty()) {
            System.out.println("RRRArmCouple12: infeasible " + StrUtil.poseStr(p));
            return q0;
        }
        RRRConfig qBest = RRRConfig.getBest(qFeasible, q0);
        System.out.printf("RRRArmCouple12: qBest %s\n", qBest);
        return qBest;
    }

    public RRRVelocity qdot(RRRConfig q, VelocitySE2 xdot) {
        return m_kinematics.inverse(q, xdot);
    }

    public RRRAcceleration qddot(RRRConfig q, VelocitySE2 xdot, AccelerationSE2 xddot) {
        return m_kinematics.inverse(q, xdot, xddot);
    }

    /** Current measured configuration. */
    @Override
    public RRRConfig getConfig() {
        // q2 kinematic angle is the difference between mechanism angles
        return new RRRConfig(
                m_q1.getUnwrappedPositionRad(),
                m_q2.getUnwrappedPositionRad() - m_q1.getUnwrappedPositionRad(),
                m_q3.getUnwrappedPositionRad());
    }

    /** Desired config, with limits applied. */
    public RRRConfig getConfigWithinLimits() {
        // q2 kinematic angle is the difference between mechanism angles
        return new RRRConfig(
                m_q1.getUnwrappedPositionWithinLimits(),
                m_q2.getUnwrappedPositionWithinLimits() - m_q1.getUnwrappedPositionWithinLimits(),
                m_q3.getUnwrappedPositionWithinLimits());
    }

    /** Current velocity. */
    public RRRVelocity getVelocity() {
        // q2 kinematic velocity is the difference between mechanism velocities
        return new RRRVelocity(
                m_q1.getVelocityRad_S(),
                m_q2.getVelocityRad_S() - m_q1.getVelocityRad_S(),
                m_q3.getVelocityRad_S());
    }

    public Pose2d pose() {
        return pose(getConfig());
    }

    public VelocitySE2 velocity() {
        return velocity(getConfig(), getVelocity());
    }

    public Pose2d pose(RRRConfig q) {
        return m_kinematics.forward(q).p4();
    }

    public VelocitySE2 velocity(RRRConfig q, RRRVelocity qdot) {
        return m_kinematics.forward(q, qdot);
    }

    @Override
    public void stop() {
        m_q1.stop();
        m_q2.stop();
        m_q3.stop();
    }

    @Override
    public StateSE2 getState() {
        return new StateSE2(pose(), velocity());
    }

    @Override
    public List<StateR1> getStateRn() {
        RRRConfig q = getConfig();
        RRRVelocity qdot = getVelocity();
        return List.of(
                new StateR1(q.q1(), qdot.q1dot()),
                new StateR1(q.q2(), qdot.q2dot()),
                new StateR1(q.q3(), qdot.q3dot()));
    }

    @Override
    public StateSE2 set(ControlSE2 setpoint) {
        System.out.printf("RRRArmCouple12: set %s\n", StrUtil.poseStr(setpoint.pose()));
        Pose2d x = setpoint.pose();
        VelocitySE2 xdot = setpoint.velocity();
        AccelerationSE2 xddot = setpoint.acceleration();
        RRRConfig q = config(x);
        RRRVelocity qdot = qdot(q, xdot);
        RRRAcceleration qddot = qddot(q, xdot, xddot);
        RRRState s = set(q, qdot, qddot);
        return new StateSE2(pose(s.q()), velocity(s.q(), s.qdot()));
    }

    @Override
    public List<StateR1> setRn(List<ControlR1> p) {
        ControlR1 c1 = p.get(0);
        ControlR1 c2 = p.get(1);
        ControlR1 c3 = p.get(2);
        RRRConfig q = new RRRConfig(c1.x(), c2.x(), c3.x());
        RRRVelocity qdot = new RRRVelocity(c1.v(), c2.v(), c3.v());
        RRRAcceleration qddot = new RRRAcceleration(c1.a(), c2.a(), c3.a());
        RRRState s = set(q, qdot, qddot);
        return List.of(
                new StateR1(s.q().q1(), s.qdot().q1dot()),
                new StateR1(s.q().q2(), s.qdot().q2dot()),
                new StateR1(s.q().q3(), s.qdot().q3dot()));
    }
}
