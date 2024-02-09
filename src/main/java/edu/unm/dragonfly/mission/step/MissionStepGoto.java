package edu.unm.dragonfly.mission.step;

import com.fasterxml.jackson.annotation.JsonCreator;
import com.fasterxml.jackson.annotation.JsonProperty;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.node.ObjectNode;
import edu.unm.dragonfly.Fixture;
import edu.unm.dragonfly.WaypointFixture;

import java.util.*;

/**
 * @author John Ericksen
 */
public class MissionStepGoto implements MissionStep {

    private final String drone;
    private final String waypoint;
    private final boolean runPump;
    private final int pumpNum;
    private final double pumpThreshold;

    @JsonCreator
    public MissionStepGoto(@JsonProperty("drone") String drone,
                           @JsonProperty("waypoint") String waypoint,
                           @JsonProperty("runPump") boolean runPump,
                           @JsonProperty("pumpNum") int pumpNum,
                           @JsonProperty("pumpThreshold") double pumpThreshold) {
        this.drone = drone;
        this.waypoint = waypoint;
        this.runPump = runPump;
        this.pumpNum = pumpNum;
        this.pumpThreshold = pumpThreshold;
    }

    @Override
    public boolean references(Fixture fixture) {
        return fixture instanceof WaypointFixture && fixture.getName().equals(waypoint);
    }

    @Override
    public boolean appliesTo(String name) {
        return this.drone.equals(name);
    }

    @Override
    public ObjectNode toROSJson(ObjectMapper mapper, String droneName) {
        ObjectNode gotoWaypoint = mapper.createObjectNode();

        gotoWaypoint.put("msg_type", MissionStepType.GOTO_WAYPOINT.getMission_type());
        ObjectNode data = gotoWaypoint.putObject("goto_step");

        data.put("waypoint", waypoint);
        data.put("run_pump", runPump);
        data.put("pump_threshold", pumpThreshold);
        data.put("pump_num", pumpNum);

        return gotoWaypoint;
    }

    public String getDrone() {
        return drone;
    }

    public String getWaypoint() {
        return waypoint;
    }

    public boolean isRunPump() {
        return runPump;
    }

    public int getPumpNum() {
        return pumpNum;
    }

    public double getPumpThreshold() {
        return pumpThreshold;
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (o == null || getClass() != o.getClass()) return false;

        MissionStepGoto that = (MissionStepGoto) o;

        if (runPump != that.runPump) return false;
        if (pumpNum != that.pumpNum) return false;
        if (Double.compare(pumpThreshold, that.pumpThreshold) != 0) return false;
        if (!Objects.equals(drone, that.drone)) return false;
        return Objects.equals(waypoint, that.waypoint);
    }

    @Override
    public int hashCode() {
        int result;
        long temp;
        result = drone != null ? drone.hashCode() : 0;
        result = 31 * result + (waypoint != null ? waypoint.hashCode() : 0);
        result = 31 * result + (runPump ? 1 : 0);
        result = 31 * result + pumpNum;
        temp = Double.doubleToLongBits(pumpThreshold);
        result = 31 * result + (int) (temp ^ (temp >>> 32));
        return result;
    }

    @Override
    public String toString() {
        return "MissionStepGoto{" +
                "drone='" + drone + '\'' +
                ", waypoint='" + waypoint + '\'' +
                ", runPump=" + runPump +
                ", pumpNum=" + pumpNum +
                ", pumpThreshold=" + pumpThreshold +
                '}';
    }
}
