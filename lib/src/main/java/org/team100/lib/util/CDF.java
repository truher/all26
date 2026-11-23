package org.team100.lib.util;

import java.util.TreeMap;

/**
 * Cumulative Distribution Function
 */
public class CDF {
    private final TreeMap<Double, Long> f;
    double n = 0;

    public CDF() {
        f = new TreeMap<>();
    }

    public void add(double x) {
        f.merge(x, 1L, Long::sum);
        n++;
    }

    public double get(double x) {
        if (n == 0) {
            return 0.0;
        }
        long cumulativeCount = f.headMap(x, true).values().stream().reduce(Long::sum).orElse(0l);
        return (double) cumulativeCount / n;
    }
}
