package org.team100.lib.uncertainty;

import org.wpilib.math.geometry.Transform3d;

/**
 * @param transform expected value
 * @param sigma     standard deviations, in order (x, y, z, rx, ry, rz)
 */
public record NoisyTransform3d(Transform3d transform, double[] sigma) {

}
