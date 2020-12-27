package edu.unm.dragonfly.mission.step;

import java.util.List;

/**
 * @author John Ericksen
 */
public class MissionStepTakeoff implements MissionStep {

    private List<String> drones;
    private double altitude;

    public MissionStepTakeoff() {
        // Empty bean constructor
    }

    public MissionStepTakeoff(List<String> drones, double altitude) {
        this.drones = drones;
        this.altitude = altitude;
    }

    public List<String> getDrones() {
        return drones;
    }

    public double getAltitude() {
        return altitude;
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (o == null || getClass() != o.getClass()) return false;

        MissionStepTakeoff that = (MissionStepTakeoff) o;

        if (Double.compare(that.altitude, altitude) != 0) return false;
        return drones != null ? drones.equals(that.drones) : that.drones == null;
    }

    @Override
    public int hashCode() {
        int result;
        long temp;
        result = drones != null ? drones.hashCode() : 0;
        temp = Double.doubleToLongBits(altitude);
        result = 31 * result + (int) (temp ^ (temp >>> 32));
        return result;
    }

    @Override
    public String toString() {
        return "Takeoff{" +
                "drones=" + drones +
                ", altitude=" + altitude +
                '}';
    }
}
