package edu.unm.dragonfly.mission.step;

import com.fasterxml.jackson.annotation.JsonCreator;
import com.fasterxml.jackson.annotation.JsonProperty;

import java.util.List;

/**
 * @author John Ericksen
 */
public class MissionStepTakeoff implements MissionStep {

    private final List<String> drones;
    private final double altitude;

    @JsonCreator
    public MissionStepTakeoff(@JsonProperty("drones") List<String> drones, @JsonProperty("altitude") double altitude) {
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
