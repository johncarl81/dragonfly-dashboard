package edu.unm.dragonfly.mission;

import com.fasterxml.jackson.databind.ObjectMapper;
import edu.unm.dragonfly.NavigateWaypoint;
import edu.unm.dragonfly.Walk;
import edu.unm.dragonfly.mission.step.MissionStart;
import edu.unm.dragonfly.mission.step.MissionStep;
import edu.unm.dragonfly.mission.step.MissionStepDDSA;
import edu.unm.dragonfly.mission.step.MissionStepGoto;
import edu.unm.dragonfly.mission.step.MissionStepLand;
import edu.unm.dragonfly.mission.step.MissionStepLawnmower;
import edu.unm.dragonfly.mission.step.MissionStepRTL;
import edu.unm.dragonfly.mission.step.MissionStepSemaphore;
import edu.unm.dragonfly.mission.step.MissionStepSleep;
import edu.unm.dragonfly.mission.step.MissionStepTakeoff;
import org.junit.Test;

import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import static org.junit.Assert.assertEquals;

/**
 * @author John Ericksen
 */
public class MissionDataHolderTest {

    @Test
    public void testMissionStepSerialization() throws IOException {
        ObjectMapper mapper = new ObjectMapper();

        List<MissionStep> steps = new ArrayList<>();

        steps.add(new MissionStart());
        steps.add(new MissionStepTakeoff(Arrays.asList("one", "two"), 3.1415));
        steps.add(new MissionStepSleep(Arrays.asList("one"), 42));
        steps.add(new MissionStepLand(Arrays.asList("two")));
        steps.add(new MissionStepGoto("three", "waypoint 1"));
        steps.add(new MissionStepRTL(Arrays.asList("one")));
        steps.add(new MissionStepSemaphore(Arrays.asList("one")));
        steps.add(new MissionStepDDSA("four", 1, 2, 3, 4, 5, Walk.RANGE, 10, 20));
        steps.add(new MissionStepLawnmower("four", 5, 4, 3, true, 1, 5, 3));

        Map<String, NavigateWaypoint> waypoints = new HashMap<>();

        waypoints.put("One", new NavigateWaypoint(new Waypoint(1, 2, 3), 42));
        waypoints.put("Two", new NavigateWaypoint(new Waypoint(3, 4, 5), 3));

        Map<String, List<Waypoint>> boundaries = new HashMap<>();
        List<Waypoint> boundaryList = new ArrayList<>();
        boundaryList.add(new Waypoint(1, 2, 3));
        boundaryList.add(new Waypoint(1, 2, 4));
        boundaryList.add(new Waypoint(4, 2, 3));
        boundaries.put("bounary", boundaryList);

        MissionDataHolder holder = new MissionDataHolder(steps, waypoints, boundaries);

        ByteArrayOutputStream outputStream = new ByteArrayOutputStream();
        mapper.writeValue(outputStream, holder);

        MissionDataHolder result = mapper.readValue(outputStream.toByteArray(), MissionDataHolder.class);

        assertEquals(steps, result.getSteps());
        assertEquals(waypoints, result.getWaypoints());
        assertEquals(boundaries, result.getBoundaries());
    }
}
