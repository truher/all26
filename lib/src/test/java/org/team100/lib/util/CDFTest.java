package org.team100.lib.util;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

public class CDFTest {
    @Test
    void test0() {
        CDF cdf = new CDF();
        cdf.add(1);
        assertEquals(0, cdf.get(0), 1e-3);
        assertEquals(1.0, cdf.get(1), 1e-3);
    }

    @Test
    void test1() {
        CDF cdf = new CDF();
        for (double x = 0.1; x < 1; x += 0.1) {
            cdf.add(x);
        }
        assertEquals(0.5, cdf.get(0.5), 1e-3);
        assertEquals(1.0, cdf.get(1), 1e-3);
    }

    @Test
    void test2() {
        CDF cdf = new CDF();
        cdf.add(1);
        cdf.add(1);
        cdf.add(2);
        cdf.add(3);
        assertEquals(0, cdf.get(0), 1e-3);
        assertEquals(0.5, cdf.get(1), 1e-3);
        assertEquals(0.75, cdf.get(2), 1e-3);
        assertEquals(1, cdf.get(3), 1e-3);
    }

}
