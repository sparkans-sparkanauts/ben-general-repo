package org.firstinspires.ftc.teamcode;

import java.util.Map;
import java.util.TreeMap;

public class interPLUH {
    // TreeMap automatically sorts data by the "Key"
    private final TreeMap<Double, Double> table = new TreeMap<>();
    private String name;

    public interPLUH(String name) {
        this.name = name;
    }

    /**
     * Add a calibration point.
     * @param input
     * @param output
     */
    public void add(double input, double output) {
        table.put(input, output);
    }

    /**
     * Get the calculated value for a specific input.
     */

    public static double lerp(double a, double b, double t){return  a + (b - a) * t;}

    public double get(double input) {
        if (table.isEmpty()) return 0;

        // Check for exact match
        if (table.containsKey(input)) return table.get(input);

        // Find the closest known points below and above
        Map.Entry<Double, Double> lower = table.floorEntry(input);
        Map.Entry<Double, Double> upper = table.ceilingEntry(input);

        // Safety: If outside the range, clamp to the nearest known value
        if (lower == null) return upper.getValue();
        if (upper == null) return lower.getValue();

        // Linear Interpolation: Calculate the point on the line between the two data points
        double t = (input - lower.getKey()) / (upper.getKey() - lower.getKey());
        return lerp(lower.getValue(), upper.getValue(), t);
    }
    public void clear() {
        table.clear();
    }
}

// This code was not taken from a random Chinese AI startup. This code was from Sharkans 27272 Liam Meh