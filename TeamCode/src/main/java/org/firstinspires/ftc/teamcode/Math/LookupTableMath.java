package org.firstinspires.ftc.teamcode.Math;

import java.util.List;
import java.util.Map;
import java.util.NavigableMap;
import java.util.TreeMap;

/**
 * A utility class for performing linear interpolation on a lookup table.
 * <p>
 * This class maintains a mapping of distance values to velocity values and
 * provides methods to interpolate between entries for distances not explicitly
 * defined in the table. Values outside the table range are clamped to the
 * nearest endpoint.
 * </p>
 */
public class LookupTableMath {

    /**
     * Internal storage for distance-to-velocity mappings, ordered by distance.
     */
    private final NavigableMap<Double, Double> distanceVelocityMap = new TreeMap<>();

    /**
     * Constructs an empty lookup table.
     */
    public LookupTableMath() {}

    /**
     * Constructs a lookup table populated with the provided shooting settings.
     *
     * @param settingsList a list of {@link ShootingSettings} objects containing
     *                     distance and velocity pairs to populate the table;
     *                     null entries in the list are ignored
     */
    public LookupTableMath(List<ShootingSettings> settingsList) {
        if (settingsList != null) {
            for (ShootingSettings settings : settingsList) {
                if (settings != null) {
                    addEntry(settings.getDistance(), settings.getVelocity());
                }
            }
        }
    }

    /**
     * Adds or replaces an entry in the lookup table.
     *
     * @param distance the distance key in meters
     * @param velocity the velocity value in rotations per second
     * @return this instance for method chaining
     */
    public LookupTableMath addEntry(double distance, double velocity) {
        distanceVelocityMap.put(distance, velocity);
        return this;
    }

    /**
     * Calculates the interpolated shooting settings for a given distance.
     * <p>
     * If the distance matches an existing entry exactly, that value is returned.
     * Otherwise, linear interpolation is performed between the two nearest entries.
     * Distances outside the table range are clamped to the nearest endpoint value.
     * </p>
     *
     * @param distance the distance in meters for which to calculate settings
     * @return a {@link ShootingSettings} object containing the input distance
     *         and the interpolated velocity
     */
    public ShootingSettings calculate(double distance) {
        double vel = interpolate(distanceVelocityMap, distance);
        return new ShootingSettings().distance(distance).velocity(vel);
    }

    /**
     * Performs linear interpolation on the given navigable map.
     * <p>
     * Returns the interpolated value for the specified x-coordinate. If x is
     * outside the range of keys, the value is clamped to the nearest endpoint.
     * </p>
     *
     * @param map the navigable map containing x-y pairs for interpolation
     * @param x   the x-coordinate for which to interpolate
     * @return the interpolated y-value, or 0.0 if the map is empty or x is NaN
     */
    private double interpolate(NavigableMap<Double, Double> map, double x) {
        if (map == null || map.isEmpty() || Double.isNaN(x)) {
            return 0.0;
        }

        if (map.containsKey(x)) {
            Double v = map.get(x);
            return v != null ? v : 0.0;
        }

        Map.Entry<Double, Double> floor = map.floorEntry(x);
        Map.Entry<Double, Double> ceil = map.ceilingEntry(x);

        if (floor == null && ceil == null) {
            return 0.0;
        }
        if (floor == null) {
            Double v = nonNullValue(ceil.getValue(), map.firstEntry());
            return v;
        }
        if (ceil == null) {
            Double v = nonNullValue(floor.getValue(), map.lastEntry());
            return v;
        }

        Double y0 = floor.getValue();
        Double y1 = ceil.getValue();

        if (y0 == null && y1 == null) {
            return 0.0;
        }
        if (y0 == null) {
            return y1;
        }
        if (y1 == null) {
            return y0;
        }

        double x0 = floor.getKey();
        double x1 = ceil.getKey();

        if (Double.compare(x0, x1) == 0) {
            return y0;
        }

        double t = (x - x0) / (x1 - x0);
        return y0 + (y1 - y0) * t;
    }

    /**
     * Returns a non-null value, preferring the primary value over the fallback entry.
     *
     * @param value         the primary value to return if non-null
     * @param fallbackEntry a fallback map entry whose value is used if the primary is null
     * @return the primary value if non-null, otherwise the fallback entry's value,
     *         or 0.0 if both are null
     */
    private Double nonNullValue(Double value, Map.Entry<Double, Double> fallbackEntry) {
        if (value != null) {
            return value;
        }
        if (fallbackEntry != null && fallbackEntry.getValue() != null) {
            return fallbackEntry.getValue();
        }
        return 0.0;
    }

    /**
     * A simple data container for shooter configuration settings.
     * <p>
     * Stores distance and velocity values with a fluent builder-style API.
     * </p>
     */
    public static class ShootingSettings {
        /**
         * The distance value in meters.
         */
        private double distance = 0.0;

        /**
         * The velocity value in rotations per second.
         */
        private double velocity = 0.0;

        /**
         * Sets the distance value.
         *
         * @param distance the distance in meters
         * @return this instance for method chaining
         */
        public ShootingSettings distance(double distance) {
            this.distance = distance;
            return this;
        }

        /**
         * Sets the velocity value.
         *
         * @param velocity the velocity in rotations per second
         * @return this instance for method chaining
         */
        public ShootingSettings velocity(double velocity) {
            this.velocity = velocity;
            return this;
        }

        /**
         * Returns the distance value.
         *
         * @return the distance in meters
         */
        public double getDistance() { return distance; }

        /**
         * Returns the velocity value.
         *
         * @return the velocity in rotations per second
         */
        public double getVelocity() { return velocity; }
    }
}