package org.team100.lib.gtsam;

import org.junit.jupiter.api.Test;
import org.wpilib.math.geometry.Pose2d;

import gtsam.GaussNewtonOptimizer;
import gtsam.GaussNewtonParams;
import gtsam.Key;
import gtsam.NonlinearFactorGraph;
import gtsam.Values;

public class SimTest {
    @Test
    void testSim() {
        System.out.println("========= construct sim ==========");
        Sim sim = Sim.make();
        System.out.println("========= run sim ==========");
        for (double t = 0; t < 1; t += 0.02) {
            System.out.printf("t %f\n", t);
            sim.run();
        }
        System.out.println("========= done! ==========");
    }

    @Test
    void testIndeterminate() throws Throwable {
        Sim sim = Sim.make();
        long t0_ns = System.nanoTime();
        long t1_us = 20000;
        Pose2d groundTruthPose = sim.m_simulatedRobot.pose(t1_us);
        Key x1 = Key.X(t1_us);
        sim.m_solver.addVariable(x1, t1_us, sim.m_estimatedPose);
        sim.applyOdometry(t1_us, groundTruthPose);
        sim.applyBetweenGyro(t1_us, groundTruthPose);
        sim.applyCamera(t1_us, groundTruthPose);
        NonlinearFactorGraph graph = sim.m_solver.m_newFactors;
        Values values = sim.m_solver.m_newValues;
        GaussNewtonParams parameters = new GaussNewtonParams();
        GaussNewtonOptimizer opt = new GaussNewtonOptimizer(graph, values, parameters);
        Values result = opt.optimize();
        result.print("result");
    }
}
