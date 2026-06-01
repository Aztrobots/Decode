package org.firstinspires.ftc.teamcode.tools;

import java.util.TreeMap;


public class LUT {

    // Zumito, cesto es como el de FRC, sólo hay que tunearlo, a ver si alcanzas antes del martes que vaya para calar el SOTM (es exactamente lo mismo que el InterpolationTreeMap)
    private final TreeMap<Double, Double> table = new TreeMap<>();

    public void addData(double key, double value) {
        table.put(key, value);
    }


    public double getValue(double key) {
        if (table.isEmpty()) return 0.0;
        if (table.size() == 1) return table.firstEntry().getValue();

        // Clamp inferior
        if (key <= table.firstKey()) return table.firstEntry().getValue();
        // Clamp superior
        if (key >= table.lastKey())  return table.lastEntry().getValue();

        // Interpolación lineal entre floor y ceiling
        Double lo = table.floorKey(key);
        Double hi = table.ceilingKey(key);

        if (lo.equals(hi)) return table.get(lo);

        double t = (key - lo) / (hi - lo);
        return table.get(lo) + t * (table.get(hi) - table.get(lo));
    }
}