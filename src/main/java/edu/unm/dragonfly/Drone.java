package edu.unm.dragonfly;

import com.esri.arcgisruntime.geometry.Point;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import edu.unm.dragonfly.msgs.DDSARequest;
import edu.unm.dragonfly.msgs.DDSAResponse;
import edu.unm.dragonfly.msgs.DDSAWaypointsRequest;
import edu.unm.dragonfly.msgs.LatLon;
import edu.unm.dragonfly.msgs.LawnmowerRequest;
import edu.unm.dragonfly.msgs.LawnmowerResponse;
import edu.unm.dragonfly.msgs.LawnmowerWaypointsRequest;
import edu.unm.dragonfly.msgs.LawnmowerWaypointsResponse;
import edu.unm.dragonfly.msgs.NavSatFix;
import edu.unm.dragonfly.msgs.NavigationRequest;
import edu.unm.dragonfly.msgs.NavigationResponse;
import edu.unm.dragonfly.msgs.PoseStamped;
import io.reactivex.Observable;
import io.reactivex.Single;
import io.reactivex.subjects.BehaviorSubject;
import io.reactivex.subjects.PublishSubject;
import io.reactivex.subjects.SingleSubject;
import ros.RosBridge;
import ros.RosListenDelegate;
import ros.msgs.std_msgs.PrimitiveMsg;
import ros.tools.MessageUnpacker;

import java.util.List;
import java.util.function.Function;
import java.util.stream.Collectors;

public class Drone {

    private final String name;
    private final RosBridge bridge;
    private final BehaviorSubject<NavSatFix> position = BehaviorSubject.create();
    private final BehaviorSubject<PoseStamped> localPosition = BehaviorSubject.create();
    private final PublishSubject<String> logSubject = PublishSubject.create();
    private final Observable<LatLonRelativeAltitude> relativeAltitudeObservable;

    public Drone(RosBridge bridge, String name) {
        this.bridge = bridge;
        this.name = name;

        relativeAltitudeObservable = Observable.combineLatest(position, localPosition,
                (navSatFix, poseStamped) -> new LatLonRelativeAltitude(navSatFix.latitude, navSatFix.longitude, poseStamped.pose.position.z));
    }

    public void init() {

        bridge.subscribe("/" + name + "/mavros/global_position/global", "sensor_msgs/NavSatFix",
                new RosListenDelegate() {
            private final MessageUnpacker<NavSatFix> unpacker = new MessageUnpacker<>(NavSatFix.class);
            @Override
            public void receive(JsonNode data, String stringRep) {
                NavSatFix msg = unpacker.unpackRosMessage(data);
                position.onNext(msg);
            }
        });

        bridge.subscribe("/" + name + "/mavros/local_position/pose", "geometry_msgs/PoseStamped",
                new RosListenDelegate() {
            private final MessageUnpacker<PoseStamped> unpacker = new MessageUnpacker<>(PoseStamped.class);
            @Override
            public void receive(JsonNode data, String stringRep) {
                PoseStamped msg = unpacker.unpackRosMessage(data);
                localPosition.onNext(msg);
            }
        });

        bridge.subscribe("/" + name + "/log", "std_msgs/String",
                new RosListenDelegate() {
            private final MessageUnpacker<PrimitiveMsg<String>> unpacker = new MessageUnpacker<>(PrimitiveMsg.class);
            @Override
            public void receive(JsonNode data, String stringRep) {

                PrimitiveMsg<String> msg = unpacker.unpackRosMessage(data);
                logSubject.onNext(msg.data);
            }
        });
    }

    public void lawnmower(List<Point> boundaryPoints, float stepLength, float altitude, int stacks, boolean walkBoundary, int walk, float waittime, float distanceThreshold) {
        LawnmowerRequest request = new LawnmowerRequest();

        request.setBoundary(boundaryPoints.stream().map(mapToLatLon()).collect(Collectors.toList()));
        request.setSteplength(stepLength);
        request.setWalkBoundary(walkBoundary);
        request.setStacks(stacks);
        request.setAltitude(altitude);
        request.setWalk(walk);
        request.setWaittime(waittime);
        request.setDistanceThreshold(distanceThreshold);

        bridge.call("/" + name + "/command/lawnmower", "dragonfly_messages/Lawnmower", request,
                new RosListenDelegate() {
                    private final MessageUnpacker<LawnmowerResponse> unpacker = new MessageUnpacker<>(LawnmowerResponse.class);
                    @Override
                    public void receive(JsonNode data, String stringRep) {
                    }
                });
    }

    public Single<List<Point>> getLawnmowerWaypoints(List<Point> boundaryPoints, float stepLength, float altitude, int stacks, boolean walkBoundary, int walk, float waittime) {
        LawnmowerWaypointsRequest request = new LawnmowerWaypointsRequest();

        request.setBoundary(boundaryPoints.stream().map(mapToLatLon()).collect(Collectors.toList())
        );
        request.setSteplength(stepLength);
        request.setWalkBoundary(walkBoundary);
        request.setStacks(stacks);
        request.setAltitude(altitude);
        request.setWalk(walk);
        request.setWaittime(waittime);

        SingleSubject<List<Point>> result = SingleSubject.create();

        bridge.call("/" + name + "/build/lawnmower", "dragonfly_messages/LawnmowerWaypoints", request,
                new RosListenDelegate() {
                    private final MessageUnpacker<LawnmowerWaypointsResponse> unpacker = new MessageUnpacker<>(LawnmowerWaypointsResponse.class);
                    @Override
                    public void receive(JsonNode data, String stringRep) {
                        LawnmowerWaypointsResponse response = unpacker.unpackRosMessage(data);
                        result.onSuccess(response.getWaypoints().stream().map(mapToPoint()).collect(Collectors.toList()));
                    }
                });

        return result;
    }

    private Function<Point, LatLon> mapToLatLon() {
        return input -> {
            LatLon position = new LatLon();
            position.setLatitude(input.getY());
            position.setLongitude(input.getX());
            position.setRelativeAltitude(input.getZ());
            return position;
        };
    }

    private Function<LatLon, Point> mapToPoint() {
        return input -> new Point(input.getLongitude(), input.getLatitude(), input.getRelativeAltitude());
    }

    public void ddsa(float radius, float stepLength, float altitude, int loops, int stacks, int walk, float waittime, float distanceThreshold) {
        DDSARequest request = new DDSARequest();
        request.setRadius(radius);
        request.setSteplength(stepLength);
        request.setStacks(stacks);
        request.setAltitude(altitude);
        request.setWalk(walk);
        request.setWaittime(waittime);
        request.setLoops(loops);
        request.setDistanceThreshold(distanceThreshold);

        bridge.call("/" + name + "/command/ddsa", "dragonfly_messages/DDSA", request,
                new RosListenDelegate() {
                    private final MessageUnpacker<DDSAResponse> unpacker = new MessageUnpacker<>(DDSAResponse.class);
                    @Override
                    public void receive(JsonNode data, String stringRep) {
                    }
                });

    }

    public Single<List<Point>> getDDSAWaypoints(float radius, float stepLength, float altitude, int loops, int stacks, int walk, float waittime) {
        DDSAWaypointsRequest request = new DDSAWaypointsRequest();
        request.setRadius(radius);
        request.setSteplength(stepLength);
        request.setStacks(stacks);
        request.setAltitude(altitude);
        request.setWalk(walk);
        request.setWaittime(waittime);
        request.setLoops(loops);

        SingleSubject<List<Point>> result = SingleSubject.create();

        bridge.call("/" + name + "/build/ddsa", "dragonfly_messages/DDSAWaypoints", request,
                new RosListenDelegate() {
                    private final MessageUnpacker<LawnmowerWaypointsResponse> unpacker = new MessageUnpacker<>(LawnmowerWaypointsResponse.class);
                    @Override
                    public void receive(JsonNode data, String stringRep) {
                        LawnmowerWaypointsResponse response = unpacker.unpackRosMessage(data);
                        result.onSuccess(response.getWaypoints().stream().map(mapToPoint()).collect(Collectors.toList()));
                    }
                });

        return result;
    }

    public void navigate(List<Point> waypoints, float distanceThreshold) {
        NavigationRequest request = new NavigationRequest();


        request.setWaypoints(waypoints.stream().map(mapToLatLon()).collect(Collectors.toList()));
        request.setWaittime(0);
        request.setDistanceThreshold(distanceThreshold);

        bridge.call("/" + name + "/command/navigate", "dragonfly_messages/Navigation", request,
                new RosListenDelegate() {
                    private final MessageUnpacker<NavigationResponse> unpacker = new MessageUnpacker<>(NavigationResponse.class);
                    @Override
                    public void receive(JsonNode data, String stringRep) {
                    }
                });
    }

    public void cancel() {

        bridge.call("/" + name + "/command/cancel", "std_msgs/Empty", null,
                new RosListenDelegate() {
                    @Override
                    public void receive(JsonNode data, String stringRep) {
                        System.out.println("Cancel sent to " + name);
                    }
                });
    }

    @Override
    public String toString() {
        return name;
    }

    public String getName() {
        return name;
    }

    public Observable<String> getLog() {
        return logSubject;
    }

    public Observable<LatLonRelativeAltitude> getLatestPosition() {
        return relativeAltitudeObservable.take(1);
    }

    public Observable<LatLonRelativeAltitude> getPositions() {
        return relativeAltitudeObservable;
    }

    public void shutdown() {
        position.onComplete();
        localPosition.onComplete();
        logSubject.onComplete();
    }

    public void takeoff() {
        bridge.call("/" + name + "/command/takeoff", "std_msgs/Empty", null,
                new RosListenDelegate() {
                    @Override
                    public void receive(JsonNode data, String stringRep) {
                        System.out.println("Takeoff sent to " + name);
                    }
                });
    }

    public void land() {
        bridge.call("/" + name + "/command/land", "std_msgs/Empty", null,
                new RosListenDelegate() {
                    @Override
                    public void receive(JsonNode data, String stringRep) {
                        System.out.println("Land sent to " + name);
                    }
                });
    }

    public void rtl() {
        bridge.call("/" + name + "/command/rtl", "std_msgs/Empty", null,
                new RosListenDelegate() {
                    @Override
                    public void receive(JsonNode data, String stringRep) {
                        System.out.println("RTL sent to " + name);
                    }
                });
    }

    public void sendMission(ObjectNode missionDataHolder) {
        bridge.call("/" + name + "/command/mission", "dragonfly_messages/Mission", missionDataHolder,
                new RosListenDelegate() {
                    @Override
                    public void receive(JsonNode data, String stringRep) {
                        System.out.println("Mission sent to " + name);
                    }
                });
    }

    public void startMission() {
        bridge.call("/" + name + "/command/start_mission", "std_msgs/Empty", null,
                new RosListenDelegate() {
                    @Override
                    public void receive(JsonNode data, String stringRep) {
                        System.out.println("Start mission sent to " + name);
                    }
                });
    }

    public static class LatLonRelativeAltitude {
        private final double latitude;
        private final double longitude;
        private final double relativeAltitude;

        public LatLonRelativeAltitude(double latitude, double longitude, double relativeAltitude) {
            this.latitude = latitude;
            this.longitude = longitude;
            this.relativeAltitude = relativeAltitude;
        }

        public double getLatitude() {
            return latitude;
        }

        public double getLongitude() {
            return longitude;
        }

        public double getRelativeAltitude() {
            return relativeAltitude;
        }
    }
}
