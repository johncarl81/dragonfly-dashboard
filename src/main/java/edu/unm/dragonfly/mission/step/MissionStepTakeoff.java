package edu.unm.dragonfly.mission.step;

import com.fasterxml.jackson.annotation.JsonCreator;
import com.fasterxml.jackson.annotation.JsonProperty;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.node.ObjectNode;
import edu.unm.dragonfly.Fixture;

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
    public boolean references(Fixture fixture) {
        return false;
    }

    @Override
    public boolean appliesTo(String name) {
        return this.drones.contains(name);
    }

    @Override
    public ObjectNode toROSJson(ObjectMapper mapper, String droneName) {
        ObjectNode takeoff = mapper.createObjectNode();

        takeoff.put("msg_type", MissionStepType.TAKEOFF.getMission_type());
        ObjectNode data = takeoff.putObject("takeoff_step");

        data.put("altitude", altitude);

        return takeoff;
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
