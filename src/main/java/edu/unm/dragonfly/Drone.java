package edu.unm.dragonfly;

import com.esri.arcgisruntime.geometry.Point;
import com.fasterxml.jackson.databind.JsonNode;
import edu.unm.dragonfly.msgs.NavSatFix;
import edu.unm.dragonfly.msgs.PoseStamped;
import io.reactivex.Observable;
import io.reactivex.Single;
import io.reactivex.subjects.BehaviorSubject;
import io.reactivex.subjects.PublishSubject;
import ros.RosBridge;
import ros.RosListenDelegate;
import ros.SubscriptionRequestMsg;
import ros.msgs.std_msgs.PrimitiveMsg;
import ros.tools.MessageUnpacker;

import java.util.List;

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

        bridge.subscribe(SubscriptionRequestMsg.generate("/" + name + "/mavros/global_position/global")
                .setType("geometry_msgs/PoseStamped"), new RosListenDelegate() {
            private final MessageUnpacker<NavSatFix> unpacker = new MessageUnpacker<>(NavSatFix.class);
            @Override
            public void receive(JsonNode data, String stringRep) {
                System.out.println(data);
                System.out.println("String: " + stringRep);
                NavSatFix msg = unpacker.unpackRosMessage(data);
                position.onNext(msg);
            }
        });

        bridge.subscribe(SubscriptionRequestMsg.generate("/" + name + "/mavros/local_position/pose")
                .setType("geometry_msgs/PoseStamped"), new RosListenDelegate() {
            private final MessageUnpacker<PoseStamped> unpacker = new MessageUnpacker<>(PoseStamped.class);
            @Override
            public void receive(JsonNode data, String stringRep) {
                System.out.println(data);
                System.out.println("String: " + stringRep);
                PoseStamped msg = unpacker.unpackRosMessage(data);
                localPosition.onNext(msg);
            }
        });

        bridge.subscribe(SubscriptionRequestMsg.generate("/" + name + "/log")
                .setType("std_msgs/String"), new RosListenDelegate() {
            private final MessageUnpacker<PrimitiveMsg<String>> unpacker = new MessageUnpacker<>(PrimitiveMsg.class);
            @Override
            public void receive(JsonNode data, String stringRep) {

                PrimitiveMsg<String> msg = unpacker.unpackRosMessage(data);
                logSubject.onNext(msg.data);
            }
        });
    }

    public void lawnmower(List<Point> boundaryPoints, float stepLength, float altitude, int stacks, boolean walkBoundary, int walk, float waittime, float distanceThreshold) {
//        ServiceClient<LawnmowerRequest, LawnmowerResponse> client = node.newServiceClient(name + "/command/lawnmower", Lawnmower._TYPE);
//        LawnmowerRequest request = client.newMessage();
//
//        NodeConfiguration config = NodeConfiguration.newPrivate();
//
//        request.setBoundary(boundaryPoints.stream().map(mapToLatLon(config)).collect(Collectors.toList()));
//        request.setSteplength(stepLength);
//        request.setWalkBoundary(walkBoundary);
//        request.setStacks(stacks);
//        request.setAltitude(altitude);
//        request.setWalk(walk);
//        request.setWaittime(waittime);
//        request.setDistanceThreshold(distanceThreshold);
//
//        client.call(request, new ServiceResponseListener<LawnmowerResponse>() {
//             @Override
//             public void onSuccess(LawnmowerResponse lawnmowerResponse) {
//                 System.out.println("Got: " + lawnmowerResponse.getMessage());
//             }
//
//             @Override
//             public void onFailure(RemoteException e) {
//
//             }
//         });
    }

    public Single<List<Point>> getLawnmowerWaypoints(List<Point> boundaryPoints, float stepLength, float altitude, int stacks, boolean walkBoundary, int walk, float waittime) {
//        ServiceClient<LawnmowerWaypointsRequest, LawnmowerWaypointsResponse> client = node.newServiceClient(name + "/build/lawnmower", LawnmowerWaypoints._TYPE);
//        LawnmowerWaypointsRequest request = client.newMessage();
//
//        NodeConfiguration config = NodeConfiguration.newPrivate();
//
//        request.setBoundary(boundaryPoints.stream().map(mapToLatLon(config)).collect(Collectors.toList())
//        );
//        request.setSteplength(stepLength);
//        request.setWalkBoundary(walkBoundary);
//        request.setStacks(stacks);
//        request.setAltitude(altitude);
//        request.setWalk(walk);
//        request.setWaittime(waittime);
//
//        SingleSubject<List<Point>> result = SingleSubject.create();
//
//        client.call(request, new ServiceResponseListener<LawnmowerWaypointsResponse>() {
//            @Override
//            public void onSuccess(LawnmowerWaypointsResponse response) {
//                result.onSuccess(response.getWaypoints().stream().map(mapToPoint()).collect(Collectors.toList()));
//            }
//
//            @Override
//            public void onFailure(RemoteException e) {
//
//            }
//        });
//
//        return result;
        return null;
    }

//    private Function<Point, LatLon> mapToLatLon(NodeConfiguration config) {
//        return input -> {
//            LatLon position = config.getTopicMessageFactory().newFromType(LatLon._TYPE);
//            position.setLatitude(input.getY());
//            position.setLongitude(input.getX());
//            position.setRelativeAltitude(input.getZ());
//            return position;
//        };
//    }
//
//    private Function<LatLon, Point> mapToPoint() {
//        return input -> {
//            System.out.println(new Point(input.getLongitude(), input.getLatitude(), input.getRelativeAltitude()));
//            return new Point(input.getLongitude(), input.getLatitude(), input.getRelativeAltitude());
//        };
//    }

    public void ddsa(float radius, float stepLength, float altitude, int loops, int stacks, int walk, float waittime, float distanceThreshold) {
//        ServiceClient<DDSARequest, DDSAResponse> client = node.newServiceClient(name + "/command/ddsa", DDSA._TYPE);
//        DDSARequest request = client.newMessage();
//        request.setRadius(radius);
//        request.setSteplength(stepLength);
//        request.setStacks(stacks);
//        request.setAltitude(altitude);
//        request.setWalk(walk);
//        request.setWaittime(waittime);
//        request.setLoops(loops);
//        request.setDistanceThreshold(distanceThreshold);
//
//        client.call(request, new ServiceResponseListener<DDSAResponse>() {
//            @Override
//            public void onSuccess(DDSAResponse response) {
//                System.out.println("Got: " + response.toString());
//            }
//
//            @Override
//            public void onFailure(RemoteException e) {
//
//            }
//        });

    }

    public Single<List<Point>> getDDSAWaypoints(float radius, float stepLength, float altitude, int loops, int stacks, int walk, float waittime) {
//        ServiceClient<DDSAWaypointsRequest, DDSAWaypointsResponse> client = node.newServiceClient(name + "/build/ddsa", DDSAWaypoints._TYPE);
//        DDSAWaypointsRequest request = client.newMessage();
//        request.setRadius(radius);
//        request.setSteplength(stepLength);
//        request.setStacks(stacks);
//        request.setAltitude(altitude);
//        request.setWalk(walk);
//        request.setWaittime(waittime);
//        request.setLoops(loops);
//
//        SingleSubject<List<Point>> result = SingleSubject.create();
//
//        client.call(request, new ServiceResponseListener<DDSAWaypointsResponse>() {
//            @Override
//            public void onSuccess(DDSAWaypointsResponse response) {
//                result.onSuccess(response.getWaypoints().stream().map(mapToPoint()).collect(Collectors.toList()));
//            }
//
//            @Override
//            public void onFailure(RemoteException e) {
//
//            }
//        });
//
//        return result;
        return null;
    }

    public void navigate(List<Point> waypoints, float distanceThreshold) {
//        ServiceClient<NavigationRequest, NavigationResponse> client = node.newServiceClient(name + "/command/navigate", Navigation._TYPE);
//        NavigationRequest request = client.newMessage();
//
//        NodeConfiguration config = NodeConfiguration.newPrivate();
//
//        request.setWaypoints(waypoints.stream().map(mapToLatLon(config)).collect(Collectors.toList()));
//        request.setWaittime(0);
//        request.setDistanceThreshold(distanceThreshold);
//
//        client.call(request, new ServiceResponseListener<NavigationResponse>() {
//            @Override
//            public void onSuccess(NavigationResponse response) {
//                System.out.println("Got: " + response.toString());
//            }
//
//            @Override
//            public void onFailure(RemoteException e) {
//
//            }
//        });
    }

    public void cancel() {
//        ServiceClient<Object, EmptyResponse> client = node.newServiceClient(name + "/command/cancel", EmptyRequest._TYPE);
//        Object request = client.newMessage();
//        client.call(request, new ServiceResponseListener<EmptyResponse>() {
//            @Override
//            public void onSuccess(EmptyResponse response) {
//                System.out.println("Got: " + response.toString());
//            }
//
//            @Override
//            public void onFailure(RemoteException e) {
//
//            }
//        });
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
