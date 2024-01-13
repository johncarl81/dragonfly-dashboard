package edu.unm.dragonfly.mission;

import com.fasterxml.jackson.annotation.JsonCreator;
import com.fasterxml.jackson.annotation.JsonProperty;
import edu.unm.dragonfly.NavigateWaypoint;
import edu.unm.dragonfly.mission.step.MissionStep;
import edu.unm.dragonfly.msgs.*;

import java.util.List;
import java.util.Map;

/**
 * @author John Ericksen
 */
public class MissionDataHolder {
    private final List<MissionStep> steps;
    private final Map<String, NavigateWaypoint> waypoints;
    private final Map<String, List<Waypoint>> boundaries;
    private final Map<String, Plume> plumes;

    @JsonCreator
    public MissionDataHolder(@JsonProperty("steps") List<MissionStep> steps,
                             @JsonProperty("waypoints") Map<String, NavigateWaypoint> waypoints,
                             @JsonProperty("boundaries") Map<String, List<Waypoint>> boundaries,
                             @JsonProperty("plumes") Map<String, Plume> plumes) {
        this.steps = steps;
        this.waypoints = waypoints;
        this.boundaries = boundaries;
        this.plumes = plumes;
    }

    public List<MissionStep> getSteps() {
        return steps;
    }

    public Map<String, NavigateWaypoint> getWaypoints() {
        return waypoints;
    }

    public Map<String, List<Waypoint>> getBoundaries() {
        return boundaries;
    }

    public Map<String, Plume> getPlumes() {
        return plumes;
    }
}
