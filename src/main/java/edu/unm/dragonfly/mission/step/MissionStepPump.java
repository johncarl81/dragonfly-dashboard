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
public class MissionStepPump implements MissionStep {
    private final List<String> drones;
    private final int pumpNum;

    @JsonCreator
    public MissionStepPump(@JsonProperty("drones") List<String> drones, @JsonProperty("pump_num") int pumpNum) {
        this.drones = drones;
        this.pumpNum = pumpNum;
    }

    public List<String> getDrones() {
        return drones;
    }

    public double getPumpNum() {
        return pumpNum;
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
        ObjectNode sleep = mapper.createObjectNode();

        sleep.put("msg_type", MissionStepType.PUMP.getMission_type());
        ObjectNode data = sleep.putObject("pump_step");

        data.put("pump_num", pumpNum);

        return sleep;
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (o == null || getClass() != o.getClass()) return false;

        MissionStepPump that = (MissionStepPump) o;

        if (pumpNum != that.pumpNum) return false;
        return drones != null ? drones.equals(that.drones) : that.drones == null;
    }

    @Override
    public int hashCode() {
        int result = drones != null ? drones.hashCode() : 0;
        result = 31 * result + pumpNum;
        return result;
    }

    @Override
    public String toString() {
        return "Pump{" +
                "drones=" + drones +
                ", pumpNum=" + pumpNum +
                '}';
    }
}
