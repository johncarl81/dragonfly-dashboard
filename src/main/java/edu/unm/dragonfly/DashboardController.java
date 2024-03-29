package edu.unm.dragonfly;

import com.esri.arcgisruntime.concurrent.ListenableFuture;
import com.esri.arcgisruntime.geometry.Point;
import com.esri.arcgisruntime.geometry.PointCollection;
import com.esri.arcgisruntime.geometry.Polygon;
import com.esri.arcgisruntime.geometry.PolylineBuilder;
import com.esri.arcgisruntime.geometry.SpatialReferences;
import com.esri.arcgisruntime.loadable.LoadStatus;
import com.esri.arcgisruntime.mapping.ArcGISScene;
import com.esri.arcgisruntime.mapping.ArcGISTiledElevationSource;
import com.esri.arcgisruntime.mapping.Basemap;
import com.esri.arcgisruntime.mapping.MobileScenePackage;
import com.esri.arcgisruntime.mapping.Surface;
import com.esri.arcgisruntime.mapping.view.Graphic;
import com.esri.arcgisruntime.mapping.view.GraphicsOverlay;
import com.esri.arcgisruntime.mapping.view.LayerSceneProperties;
import com.esri.arcgisruntime.mapping.view.SceneView;
import com.esri.arcgisruntime.symbology.CompositeSymbol;
import com.esri.arcgisruntime.symbology.SceneSymbol;
import com.esri.arcgisruntime.symbology.SimpleFillSymbol;
import com.esri.arcgisruntime.symbology.SimpleLineSymbol;
import com.esri.arcgisruntime.symbology.SimpleMarkerSceneSymbol;
import com.esri.arcgisruntime.symbology.SimpleMarkerSymbol;
import com.esri.arcgisruntime.symbology.TextSymbol;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.node.ArrayNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import edu.unm.dragonfly.mission.MissionDataHolder;
import edu.unm.dragonfly.mission.MissionStepDialogFactory;
import edu.unm.dragonfly.mission.PointUtil;
import edu.unm.dragonfly.mission.Waypoint;
import edu.unm.dragonfly.mission.step.MissionStart;
import edu.unm.dragonfly.mission.step.MissionStep;
import edu.unm.dragonfly.msgs.*;
import edu.unm.dragonfly.tsp.TSP;
import io.reactivex.Observable;
import io.reactivex.ObservableSource;
import io.reactivex.Observer;
import io.reactivex.disposables.Disposable;
import io.reactivex.functions.Consumer;
import io.reactivex.rxjavafx.observables.JavaFxObservable;
import io.reactivex.rxjavafx.schedulers.JavaFxScheduler;
import io.reactivex.schedulers.Schedulers;
import io.reactivex.subjects.PublishSubject;
import javafx.beans.value.ChangeListener;
import javafx.beans.value.ObservableValue;
import javafx.collections.FXCollections;
import javafx.collections.ObservableList;
import javafx.event.ActionEvent;
import javafx.event.EventHandler;
import javafx.fxml.FXML;
import javafx.geometry.Point2D;
import javafx.geometry.Pos;
import javafx.scene.control.Alert;
import javafx.scene.control.Button;
import javafx.scene.control.ButtonType;
import javafx.scene.control.ComboBox;
import javafx.scene.control.ListView;
import javafx.scene.control.TextField;
import javafx.scene.control.TextInputDialog;
import javafx.scene.input.MouseEvent;
import javafx.scene.layout.StackPane;
import javafx.stage.FileChooser;
import javafx.stage.Stage;
import ros.RosBridge;
import ros.SubscriptionRequestMsg;
import ros.msgs.std_msgs.PrimitiveMsg;

import javax.annotation.Nullable;
import javax.inject.Inject;
import javax.inject.Named;
import java.io.File;
import java.io.IOException;
import java.text.DateFormat;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.Comparator;
import java.util.Date;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.Random;
import java.util.Set;
import java.util.concurrent.Callable;
import java.util.concurrent.ExecutionException;
import java.util.concurrent.TimeUnit;
import java.util.stream.Collectors;

public class DashboardController {

    private static final int[] COLORS = {0xFF1F77B4, 0xFFFF7F0E, 0xFF2CA02C, 0xFFd62728, 0xFF9467BD, 0xFF8C564B, 0xFFE377C2, 0xFF7F7F7F, 0xFFBCBD22, 0xFF17BECF};
    private static final DateFormat DATE_FORMAT = new SimpleDateFormat("hh:mm:ss");
    private static final String ELEVATION_IMAGE_SERVICE =
            "https://elevation3d.arcgis.com/arcgis/rest/services/WorldElevation3D/Terrain3D/ImageServer";
    private static final int ALPHA_RED = 0x33FF0000;
    private static final int RED = 0xFFFF0000;
    private static final int GREEN = 0xFF00FF00;
    private static final Random RAND = new Random(System.currentTimeMillis());

    @FXML
    private StackPane mapPlaceholder;
    @FXML
    private TextField maxAltitude;
    @FXML
    private ComboBox<String> rtlBoundary;
    @FXML
    private TextField rtlAltitudeFactor;
    @FXML
    private Button menuLoadMap;
    @FXML
    private Button add;
    @FXML
    private Button delete;
    @FXML
    private Button center;
    @FXML
    private ListView<DroneStatus> drones;
    @FXML
    private ListView<Fixture> fixtures;
    @FXML
    private ListView<NamedPlume> plumesListView;
    @FXML
    private ListView<String> log;
    @FXML
    private ListView<MissionStep> mission;
    @FXML
    private TextField coordinates;
    @FXML
    private Button setupDrones;
    @FXML
    private Button select;
    @FXML
    private Button lawnmower;
    @FXML
    private Button ddsa;
    @FXML
    private Button random;
    @FXML
    private Button pump;
    @FXML
    private Button cancel;
    @FXML
    private Button waypoint;
    @FXML
    private Button takeoff;
    @FXML
    private Button land;
    @FXML
    private Button rtl;
    @FXML
    private Button deleteFixture;
    @FXML
    private Button centerFixture;
    @FXML
    private Button missionAdd;
    @FXML
    private Button missionLoad;
    @FXML
    private Button missionSave;
    @FXML
    private Button missionUpload;
    @FXML
    private Button missionStart;
    @FXML
    private Button missionClear;
    @FXML
    private Button addPlume;
    @FXML
    private Button deletePlume;
    @FXML
    private Button setupPlumes;

    @Inject
    private RosBridge bridge;
    @Inject
    private Stage stage;
    @Inject
    @Named(DashboardModule.MAP_OVERRIDE)
    @Nullable
    private String mapOverride;

    private final ObservableList<Drone> droneList = FXCollections.observableArrayList();
    private final ObservableList<DroneStatus> droneStatusList = FXCollections.observableArrayList();
    private final ObservableList<String> logList = FXCollections.observableArrayList();
    private final ObservableList<MissionStep> missionList = FXCollections.observableArrayList();
    private final ObservableList<Fixture> fixtureList = FXCollections.observableArrayList();
    private final ObservableList<NamedPlume> plumeList = FXCollections.observableArrayList();
    private final GraphicsOverlay droneOverlay = new GraphicsOverlay();
    private final GraphicsOverlay droneShadowOverlay = new GraphicsOverlay();
    private final GraphicsOverlay drawBoundaryOverlay = new GraphicsOverlay();
    private final GraphicsOverlay pathOverlay = new GraphicsOverlay();
    private final GraphicsOverlay waypointOverlay = new GraphicsOverlay();
    private final GraphicsOverlay boundaryOverlay = new GraphicsOverlay();
    private final GraphicsOverlay sketchOverlay = new GraphicsOverlay();
    private final List<Point> boundaryPoints = new ArrayList<>();
    private CoordinateSelectionMode mode = CoordinateSelectionMode.DRAW;
    private final Map<String, NavigateWaypoint> waypoints = new HashMap<>();
    private final Map<String, List<Waypoint>> boundaries = new HashMap<>();
    private final Map<String, Plume> plumes = new HashMap<>();
    SceneView sceneView;
    private boolean localMap = false;
    private boolean zoomedToFirst = false;

    private enum CoordinateSelectionMode {
        FINISHED("Finished"),
        DRAW("Draw Boundary");

        private final String buttonLabel;

        CoordinateSelectionMode(String buttonLabel) {
            this.buttonLabel = buttonLabel;
        }
    }

    private void drawBoundaries() {
        drawBoundaryOverlay.getGraphics().clear();

        Graphic boundaryGraphic;
        if(boundaryPoints.size() == 1) {
            SimpleMarkerSymbol markerSymbol = new SimpleMarkerSymbol(SimpleMarkerSymbol.Style.CIRCLE, RED, 2);
            boundaryGraphic = new Graphic(new Point(boundaryPoints.get(0).getX(), boundaryPoints.get(0).getY()), markerSymbol);
        } else if(boundaryPoints.size() == 2) {
            PolylineBuilder lineBuilder = new PolylineBuilder(SpatialReferences.getWgs84());
            lineBuilder.addPoint(boundaryPoints.get(0).getX(), boundaryPoints.get(0).getY());
            lineBuilder.addPoint(boundaryPoints.get(1).getX(), boundaryPoints.get(1).getY());
            SimpleLineSymbol lineSymbol = new SimpleLineSymbol(SimpleLineSymbol.Style.SOLID, RED, 1);
            boundaryGraphic = new Graphic(lineBuilder.toGeometry(), lineSymbol);
        } else {
            PointCollection polygonPoints = new PointCollection(boundaryPoints);
            SimpleFillSymbol polygonSymbol = new SimpleFillSymbol(SimpleFillSymbol.Style.SOLID, ALPHA_RED, new SimpleLineSymbol(SimpleLineSymbol.Style.SOLID, RED, .3f));
            boundaryGraphic = new Graphic(new Polygon(polygonPoints), polygonSymbol);
        }
        drawBoundaryOverlay.getGraphics().add(boundaryGraphic);
    }


    public void initialize() {

        maxAltitude.setText("100");
        rtlAltitudeFactor.setText("2");

        sceneView = new SceneView();
        mapPlaceholder.getChildren().add(sceneView);

        if(mapOverride != null) {
            loadLocalMap(sceneView, new File(mapOverride));
        } else {
            loadWebMap(sceneView);
        }



        menuLoadMap.setOnAction(new EventHandler<ActionEvent>() {
            @Override
            public void handle(ActionEvent event) {
                if(!localMap) {
                    FileChooser fileChooser = new FileChooser();
                    File openFile = fileChooser.showOpenDialog(stage);

                    if(openFile != null) {
                        loadLocalMap(sceneView, openFile);
                    }
                } else {
                    sceneView.getGraphicsOverlays().clear();
                    loadWebMap(sceneView);
                    menuLoadMap.setText("Load Scene...");
                    localMap = false;
                }
            }
        });

        select.setOnAction(new EventHandler<ActionEvent>() {
            @Override
            public void handle(ActionEvent event) {
                if (mode == CoordinateSelectionMode.FINISHED) {
                    if(boundaryPoints.size() < 3) {
                        Alert alert = new Alert(Alert.AlertType.ERROR, "Unable to create boundary", ButtonType.CANCEL, ButtonType.OK);
                        final Optional<ButtonType> result = alert.showAndWait();

                        if(result.isPresent() && result.get() == ButtonType.CANCEL) {
                            mode = CoordinateSelectionMode.DRAW;
                            drawBoundaryOverlay.getGraphics().clear();
                            boundaryPoints.clear();
                            updateBoundaryOverlay();
                        }
                    } else {
                        TextInputDialog dialog = new TextInputDialog();
                        dialog.setHeaderText("Boundary Name");
                        Optional<String> output = dialog.showAndWait();

                        if(output.isPresent()) {
                            boundaries.put(output.get(), boundaryPoints.stream().map(Waypoint::from).collect(Collectors.toList()));
                        }

                        mode = CoordinateSelectionMode.DRAW;
                        drawBoundaryOverlay.getGraphics().clear();
                        boundaryPoints.clear();
                        updateBoundaryOverlay();
                    }
                } else if (mode == CoordinateSelectionMode.DRAW) {
                    mode = CoordinateSelectionMode.FINISHED;
                }
                select.setText(mode.buttonLabel);
            }
        });

        JavaFxObservable.emitOnChanged(droneList).switchMap(new io.reactivex.functions.Function<ObservableList<Drone>, ObservableSource<List<DroneStatus>>>() {

            @Override
            public ObservableSource<List<DroneStatus>> apply(ObservableList<Drone> drones) {
                if(drones.isEmpty()) {
                    return Observable.just(Collections.emptyList());
                } else {
                    List<Observable<DroneStatus>> droneStatusStreams = drones.stream()
                            .sorted(Comparator.comparing(Drone::getName))
                            .map(drone -> drone.getStatus().map(status -> new DroneStatus(drone, status)))
                            .collect(Collectors.toList());

                    return Observable.combineLatest(droneStatusStreams, new io.reactivex.functions.Function<Object[], List<DroneStatus>>() {
                        @Override
                        public List<DroneStatus> apply(Object[] input) {
                            List<DroneStatus> statusList = new ArrayList<>();
                            for (Object value : input) {
                                statusList.add((DroneStatus) value);
                            }
                            return statusList;
                        }
                    });
                }
            }
        })
                .observeOn(JavaFxScheduler.platform())
                .subscribe(new Consumer<List<DroneStatus>>() {
            @Override
            public void accept(List<DroneStatus> droneStatuses) {
                DroneStatus selectedItem = drones.getSelectionModel().getSelectedItem();
                droneStatusList.setAll(droneStatuses);
                drones.getSelectionModel().select(selectedItem);
            }
        });

        drones.setItems(droneStatusList);
        drones.setCellFactory(new DroneCellFactory());
        fixtures.setItems(fixtureList.sorted());
        fixtures.setCellFactory(new FixtureIconCellFactory());
        plumesListView.setItems(plumeList);
        plumesListView.setCellFactory(new PlumeCellFactory());
        log.setItems(logList);
        log.setCellFactory(new TooltipCellFactory<>());
        mission.setItems(missionList);
        mission.setCellFactory(new TooltipCellFactory<>());
        clearMission();

        delete.setDisable(true);
        takeoff.setDisable(true);
        land.setDisable(true);
        rtl.setDisable(true);
        waypoint.setDisable(true);
        center.setDisable(true);
        lawnmower.setDisable(true);
        ddsa.setDisable(true);
        random.setDisable(true);
        pump.setDisable(true);
        cancel.setDisable(true);
        deleteFixture.setDisable(true);
        centerFixture.setDisable(true);

        add.setOnAction(new EventHandler<ActionEvent>() {
            @Override
            public void handle(ActionEvent event) {
                TextInputDialog dialog = new TextInputDialog("Add Drone");
                dialog.setHeaderText("Add Drone");
                Optional<String> output = dialog.showAndWait();

                if(output.isPresent()) {
                    addDrone(output.get());
                    drones.getSelectionModel().clearSelection();
                }
            }
        });

        takeoff.setOnAction(new EventHandler<ActionEvent>() {
            @Override
            public void handle(ActionEvent event) {
                Drone selected = drones.getSelectionModel().getSelectedItem().getDrone();
                selected.takeoff();
            }
        });

        land.setOnAction(new EventHandler<ActionEvent>() {
            @Override
            public void handle(ActionEvent event) {
                Drone selected = drones.getSelectionModel().getSelectedItem().getDrone();
                selected.land();
            }
        });

        rtl.setOnAction(new EventHandler<ActionEvent>() {
            @Override
            public void handle(ActionEvent event) {
                Drone selected = drones.getSelectionModel().getSelectedItem().getDrone();
                selected.rtl();
            }
        });

        waypoint.setOnAction(new EventHandler<ActionEvent>() {
            @Override
            public void handle(ActionEvent event) {
                TextInputDialog dialog = new TextInputDialog("Goto Waypoint");
                dialog.setHeaderText("Add Drone");

                Drone selected = drones.getSelectionModel().getSelectedItem().getDrone();


                AddWaypointDialogFactory.createSelect(waypoints, selected, new AddWaypointDialogFactory.NavigateWaypointCallback(){

                    @Override
                    public void call(NavigateWaypoint waypoint) {
                        selected.navigate(Collections.singletonList(waypoint.toPoint()), waypoint.getDistanceThreshold());
                    }
                });

                drones.getSelectionModel().clearSelection();
            }
        });

        delete.setOnAction(new EventHandler<ActionEvent>() {
            @Override
            public void handle(ActionEvent event) {
                deleteDrone(drones.getSelectionModel().getSelectedItem().getDrone());
                drones.getSelectionModel().clearSelection();
            }
        });

        center.setOnAction(new EventHandler<ActionEvent>() {
            @Override
            public void handle(ActionEvent event) {
                centerDrone(drones.getSelectionModel().getSelectedItem().getDrone());
                drones.getSelectionModel().clearSelection();
            }
        });

        lawnmower.setOnAction(new EventHandler<ActionEvent>() {
            @Override
            public void handle(ActionEvent event) {
                if(!boundaries.isEmpty()) {
                    LawnmowerDialogFactory.create(boundaries, (boundaryName, stepLength, altitude, stacks, walkBoundary, walk, waitTime, distanceThreshold) -> {
                        Drone selected = drones.getSelectionModel().getSelectedItem().getDrone();
                        List<Point> selecteBoundaryPoints = boundaries.get(boundaryName).stream().map(Waypoint::toPoint).collect(Collectors.toList());
                        selected.getLawnmowerWaypoints(selecteBoundaryPoints, stepLength, altitude, stacks, walkBoundary, walk.id, waitTime)
                                .observeOn(JavaFxScheduler.platform())
                                .subscribe(waypoints -> draw(waypoints));
                        selected.lawnmower(selecteBoundaryPoints, stepLength, altitude, stacks, walkBoundary, walk.id, waitTime, distanceThreshold);
                    });
                }
                drones.getSelectionModel().clearSelection();
            }
        });

        ddsa.setOnAction(new EventHandler<ActionEvent>() {
            @Override
            public void handle(ActionEvent event) {
                DDSADialogFactory.create((radius, stepLength, altitude, loops, stacks, walk, waitTime, distanceThreshold) -> {
                    Drone selected = drones.getSelectionModel().getSelectedItem().getDrone();
                    selected.getDDSAWaypoints(radius, stepLength, altitude, loops, stacks, walk.id, waitTime)
                            .observeOn(JavaFxScheduler.platform())
                            .subscribe(waypoints -> draw(waypoints));
                    selected.ddsa(radius, stepLength, altitude, loops, stacks, walk.id, waitTime, distanceThreshold);

                });
                drones.getSelectionModel().clearSelection();
            }
        });

        random.setOnAction(new EventHandler<ActionEvent>() {
            @Override
            public void handle(ActionEvent event) {
                if (!boundaries.isEmpty()) {
                    Drone selectedDrone = drones.getSelectionModel().getSelectedItem().getDrone();
                    RandomPathDialogFactory.create(boundaries, (boundaryName, minAltitude, maxAltitude, size, waitTime, distanceThreshold) -> {
                        Observable.fromCallable(new Callable<List<ProjectedPoint>>() {
                            @Override
                            public List<ProjectedPoint> call() {
                                List<ProjectedPoint> randomPoints = PointUtil.createRandomPoints(
                                        boundaries.get(boundaryName).stream().map(Waypoint::toPoint).collect(Collectors.toList()),
                                        size,
                                        minAltitude,
                                        maxAltitude);
                                return TSP.optimize(randomPoints);
                            }
                        })
                                .subscribeOn(Schedulers.computation())
                                .observeOn(JavaFxScheduler.platform())
                                .subscribe(tour -> {
                                            List<Point> points = tour.stream().map(ProjectedPoint::getOriginal).collect(Collectors.toList());
                                            draw(points);
                                            selectedDrone.navigate(points, distanceThreshold);
                                        },
                                        throwable -> throwable.printStackTrace());
                    });
                    drones.getSelectionModel().clearSelection();
                }
            }
        });

        pump.setOnAction(new EventHandler<ActionEvent>() {
            @Override
            public void handle(ActionEvent event) {
                Drone selectedDrone = drones.getSelectionModel().getSelectedItem().getDrone();
                PumpDialogFactory.create((pumpNum) -> selectedDrone.pump(pumpNum));
                drones.getSelectionModel().clearSelection();
            }
        });



        cancel.setOnAction(new EventHandler<ActionEvent>() {
            @Override
            public void handle(ActionEvent event) {
                drones.getSelectionModel().getSelectedItem().getDrone().cancel();
                drones.getSelectionModel().clearSelection();
            }
        });

        setupDrones.setOnAction(event -> setupDrones());
        setupPlumes.setOnAction(event -> setupPlumes());
        missionAdd.setOnAction(event -> addMissionStepDialog());
        missionLoad.setOnAction(event -> loadMissionFromFile());
        missionSave.setOnAction(event -> saveMissionToFile());
        missionUpload.setOnAction(event -> uploadMissionToDrones());
        missionStart.setOnAction(event -> startMission());
        missionClear.setOnAction(event -> clearMission());

        drones.getSelectionModel().selectedItemProperty().addListener(new ChangeListener<DroneStatus>() {
            @Override
            public void changed(ObservableValue observable, DroneStatus oldValue, DroneStatus newValue) {
                boolean selected = newValue != null;
                takeoff.setDisable(!selected);
                land.setDisable(!selected);
                rtl.setDisable(!selected);
                delete.setDisable(!selected);
                waypoint.setDisable(!selected);
                center.setDisable(!selected);
                lawnmower.setDisable(!(selected && !boundaries.isEmpty()));
                ddsa.setDisable(!selected);
                random.setDisable(!(selected && !boundaries.isEmpty()));
                pump.setDisable(!selected);
                cancel.setDisable(!selected);
            }
        });

        fixtures.getSelectionModel().selectedItemProperty().addListener(new ChangeListener<Fixture>() {
            @Override
            public void changed(ObservableValue observable, Fixture oldValue, Fixture newValue) {
                boolean selected = newValue != null;
                deleteFixture.setDisable(!selected || isReferenced(newValue));
                centerFixture.setDisable(!selected);
            }
        });

        deleteFixture.setOnAction(new EventHandler<ActionEvent>() {
            @Override
            public void handle(ActionEvent event) {
                Fixture selectedItem = fixtures.getSelectionModel().getSelectedItem();
                if(!isReferenced(selectedItem)) {
                    selectedItem.remove();
                    updateFixtures();
                }
            }
        });

        centerFixture.setOnAction(new EventHandler<ActionEvent>() {
            @Override
            public void handle(ActionEvent event) {
                Fixture selectedItem = fixtures.getSelectionModel().getSelectedItem();
                selectedItem.center(sceneView);
            }
        });

        plumesListView.getSelectionModel().selectedItemProperty().addListener(new ChangeListener<NamedPlume>() {
            @Override
            public void changed(ObservableValue observable, NamedPlume oldValue, NamedPlume newValue) {
                deletePlume.setDisable(newValue == null);
            }
        });

        addPlume.setOnAction(new EventHandler<ActionEvent>() {
            @Override
            public void handle(ActionEvent event) {
                PlumeDialog.create(waypoints, new PlumeDialog.PlumeFactoryCallback() {
                    @Override
                    public void call(String name, Plume plume) {
                        plumes.put(name, plume);
                        updatePlumes();
                    }
                });
            }
        });

        deletePlume.setOnAction(new EventHandler<ActionEvent>() {
            @Override
            public void handle(ActionEvent event) {
                NamedPlume selectedItem = plumesListView.getSelectionModel().getSelectedItem();
                selectedItem.delete();
            }
        });

        PublishSubject<String> nameSubject = PublishSubject.create();
        bridge.subscribe(SubscriptionRequestMsg.generate("/dragonfly/announce")
                .setType("std_msgs/String"), new JsonRosListenerDelegate<PrimitiveMsg<String>>(PrimitiveMsg.class,
                value -> nameSubject.onNext(value.data())));

        nameSubject
                .observeOn(JavaFxScheduler.platform())
                .subscribe(
                        name -> addDrone(name),
                        error -> System.out.println("error while consuming announce: " + error.getMessage()));

        log("Dashboard Startup");
    }

    private boolean isReferenced(Fixture fixture) {
        for(MissionStep step : missionList) {
            if(step.references(fixture)) {
                return true;
            }
        }
        return fixture instanceof BoundaryFixture && fixture.getName().equals(rtlBoundary.getSelectionModel().getSelectedItem());
    }

    private void loadWebMap(SceneView sceneView) {
        ArcGISScene scene = new ArcGISScene();
        scene.setBasemap(Basemap.createImagery());

        sceneView.setArcGISScene(scene);

        // add base surface for elevation data
        Surface surface = new Surface();
        surface.getElevationSources().add(new ArcGISTiledElevationSource(ELEVATION_IMAGE_SERVICE));
        scene.setBaseSurface(surface);

        initalizeScene();

    }

    private void loadLocalMap(SceneView sceneView, File mapFile) {
        sceneView.getGraphicsOverlays().clear();
        loadMSPK(mapFile, sceneView);
        menuLoadMap.setText("Toggle Web Scene");
        localMap = true;
    }

    private void loadMSPK(File mmspFile, SceneView sceneView) {
        // load a mobile scene package
        final String mspkPath = mmspFile.getAbsolutePath();
        MobileScenePackage mobileScenePackage = new MobileScenePackage(mspkPath);

        mobileScenePackage.loadAsync();
        mobileScenePackage.addDoneLoadingListener(() -> {

            if (mobileScenePackage.getLoadStatus() == LoadStatus.LOADED && mobileScenePackage.getScenes().size() > 0) {
                // set the first scene from the package to the scene view
                sceneView.setArcGISScene(mobileScenePackage.getScenes().get(0));

                initalizeScene();
                // Don't zoom if we loaded a mspk.
                zoomedToFirst = true;
            } else {
                Alert alert = new Alert(Alert.AlertType.ERROR, "Failed to load the mobile scene package");
                alert.show();
            }
        });
    }

    private void addMissionStepDialog() {
        MissionStepDialogFactory.create(missionList::add, droneList.stream().map(Drone::getName).sorted().collect(Collectors.toList()),
                new ArrayList<>(waypoints.keySet()), boundaries);
    }

    private void loadMissionFromFile() {
        FileChooser fileChooser = new FileChooser();
        File openFile = fileChooser.showOpenDialog(stage);

        if(openFile != null) {
            ObjectMapper mapper = new ObjectMapper();

            try {
                MissionDataHolder holder = mapper.readValue(openFile, MissionDataHolder.class);
                missionList.clear();
                if (holder.getSteps() != null) {
                    missionList.addAll(holder.getSteps());
                }

                if (holder.getWaypoints() != null) {
                    waypoints.putAll(holder.getWaypoints());
                    updateWaypointOverlay();
                }

                if (holder.getBoundaries() != null) {
                    boundaries.putAll(holder.getBoundaries());
                    updateBoundaryOverlay();
                }

                if (holder.getPlumes() != null) {
                    plumes.putAll(holder.getPlumes());
                    updatePlumes();
                }

            } catch (IOException e) {
                e.printStackTrace();
            }
        }
    }

    private void saveMissionToFile() {
        FileChooser fileChooser = new FileChooser();
        File saveFile = fileChooser.showSaveDialog(stage);
        if(saveFile != null) {
            ObjectMapper mapper = new ObjectMapper();

            try {
                mapper.writerWithDefaultPrettyPrinter().writeValue(saveFile, new MissionDataHolder(missionList, waypoints, boundaries, plumes));
            } catch (IOException e) {
                e.printStackTrace();
            }
        }
    }

    private void setupDrones() {
        int maxAltitudeInteger = Integer.parseInt(maxAltitude.getText());
        int rtlFactor = Integer.parseInt(rtlAltitudeFactor.getText());
        Boundary boundary = null;
        if(boundaries.containsKey(rtlBoundary.getValue())) {
            boundary = Boundary.create(rtlBoundary.getValue(),
                    boundaries.get(rtlBoundary.getValue()).stream().map(Waypoint::toLatLon).collect(Collectors.toList()));
        }

        for(int i = 0; i < droneList.size(); i++) {
            Drone drone = droneList.get(i);
            drone.setup(SetupRequest.create(1000 + (i * rtlFactor * 100),
                    maxAltitudeInteger,
                    boundary));
        }
    }

    private void setupPlumes() {
        for(int i = 0; i < droneList.size(); i++) {
            Drone drone = droneList.get(i);
            drone.setupPlumes(SetupPlumesRequest.create(new ArrayList<>(plumes.values())));
        }
    }

    private void uploadMissionToDrones() {
        ObjectMapper mapper = new ObjectMapper();
        for(Drone drone : droneList) {
            final ObjectNode mission = mapper.createObjectNode();
            ArrayNode missionSteps = mission.putArray("steps");
            for(MissionStep step : missionList){
                if(step.appliesTo(drone.getName())) {
                    missionSteps.add(step.toROSJson(mapper, drone.getName()));
                }
            }
            ArrayNode jsonWaypoints = mission.putArray("waypoints");
            for(Map.Entry<String, NavigateWaypoint> entry : waypoints.entrySet()) {
                final ObjectNode waypointNode = entry.getValue().toROSJson(mapper);
                waypointNode.put("name", entry.getKey());
                jsonWaypoints.add(waypointNode);
            }
            ArrayNode jsonBoundaries = mission.putArray("boundaries");
            for(Map.Entry<String, List<Waypoint>> entry : boundaries.entrySet()) {
                final ObjectNode boundaryNode = jsonBoundaries.addObject();
                boundaryNode.put("name", entry.getKey());
                ArrayNode pointNodes = boundaryNode.putArray("points");
                for(Waypoint waypoint : entry.getValue()) {
                    ObjectNode pointNode = pointNodes.addObject();
                    pointNode.put("longitude", waypoint.getLongitude());
                    pointNode.put("latitude", waypoint.getLatitude());
                    pointNode.put("relative_altitude", waypoint.getAltitude());
                }
                jsonBoundaries.add(boundaryNode);
            }
            drone.sendMission(mission);
        }
    }

    private void clearMission() {
        missionList.clear();
        missionList.add(new MissionStart());
    }

    private void startMission() {
        Set<Drone> includedDrones = new HashSet<>();
        for(Drone drone : droneList) {
            for(MissionStep step : missionList){
                if(step.appliesTo(drone.getName())) {
                    includedDrones.add(drone);
                }
            }
        }

        for(Drone drone : includedDrones) {
            drone.startMission();
        }
    }

    private void initalizeScene() {
        sceneView.getGraphicsOverlays().add(droneOverlay);
        droneOverlay.getSceneProperties().setSurfacePlacement(LayerSceneProperties.SurfacePlacement.RELATIVE);
        sceneView.getGraphicsOverlays().add(droneShadowOverlay);
        droneShadowOverlay.getSceneProperties().setSurfacePlacement(LayerSceneProperties.SurfacePlacement.DRAPED_FLAT);
        sceneView.getGraphicsOverlays().add(drawBoundaryOverlay);
        drawBoundaryOverlay.getSceneProperties().setSurfacePlacement(LayerSceneProperties.SurfacePlacement.DRAPED_FLAT);
        sceneView.getGraphicsOverlays().add(pathOverlay);
        pathOverlay.getSceneProperties().setSurfacePlacement(LayerSceneProperties.SurfacePlacement.RELATIVE);
        sceneView.getGraphicsOverlays().add(waypointOverlay);
        waypointOverlay.getSceneProperties().setSurfacePlacement(LayerSceneProperties.SurfacePlacement.RELATIVE);
        sceneView.getGraphicsOverlays().add(boundaryOverlay);
        boundaryOverlay.getSceneProperties().setSurfacePlacement(LayerSceneProperties.SurfacePlacement.DRAPED_FLAT);
        sceneView.getGraphicsOverlays().add(sketchOverlay);
        sketchOverlay.getSceneProperties().setSurfacePlacement(LayerSceneProperties.SurfacePlacement.DRAPED_FLAT);

        /*bridge.subscribe("/dragonfly/sketch", "dragonfly_messages/PositionVector",
                new JsonRosListenerDelegate<>(PositionVector.class, new JsonRosListenerDelegate.Receiver<PositionVector>(){

                    @Override
                    public void receive(PositionVector value) {
                        if (value.center() != null && value.center().longitude() != 0) {
                            sketchOverlay.getGraphics().clear();
                            System.out.println("Got value: " + value);
                            Point point = new Point(value.center().longitude(), value.center().latitude());
                            SimpleMarkerSymbol markerSymbol = new SimpleMarkerSymbol(SimpleMarkerSymbol.Style.CIRCLE, RED, 1);
                            Graphic center = new Graphic(point, markerSymbol);

                            sketchOverlay.getGraphics().add(center);
                        }
                    }
                }));*/

        sceneView.setOnMouseMoved(new EventHandler<MouseEvent>() {
            @Override
            public void handle(MouseEvent event) {
                Point2D point2D = new Point2D(event.getX(), event.getY());

                // get the scene location from the screen position
                ListenableFuture<Point> pointFuture = sceneView.screenToLocationAsync(point2D);
                pointFuture.addDoneListener(() -> {
                    try {
                        Point point = pointFuture.get();
                        coordinates.setText("Lat: " + String.format("%,.5f", point.getY()) + " Lon: " + String.format("%,.5f", point.getX()));

                    } catch (InterruptedException | ExecutionException e) {
                        e.printStackTrace();
                    }
                });
            }
        });

        sceneView.setOnMouseClicked(new EventHandler<MouseEvent>() {
            @Override
            public void handle(MouseEvent event) {
                if(mode == CoordinateSelectionMode.FINISHED) {
                    Point2D point2D = new Point2D(event.getX(), event.getY());

                    // get the scene location from the screen position
                    ListenableFuture<Point> pointFuture = sceneView.screenToLocationAsync(point2D);
                    pointFuture.addDoneListener(() -> {
                        try {
                            Point point = pointFuture.get();

                            boundaryPoints.add(point);

                            drawBoundaries();
                        } catch (InterruptedException | ExecutionException e) {
                            e.printStackTrace();
                        }
                    });
                } else if (event.isControlDown()) {
                    Point2D point2D = new Point2D(event.getX(), event.getY());

                    ListenableFuture<Point> pointFuture = sceneView.screenToLocationAsync(point2D);
                    pointFuture.addDoneListener(() -> {
                        try {
                            Point point = pointFuture.get();

                            AddWaypointDialogFactory.create(point, new AddWaypointDialogFactory.AddWaypointCallback() {
                                @Override
                                public void call(String name, double latitude, double longitude, double altitude, float distanceThreshold) {
                                    waypoints.put(name, new NavigateWaypoint(new Waypoint(longitude, latitude, altitude), distanceThreshold));
                                    updateWaypointOverlay();
                                }
                            });
                        } catch (InterruptedException | ExecutionException e) {
                            e.printStackTrace();
                        }
                    });
                }
            }
        });
    }

    private void updateWaypointOverlay() {
        waypointOverlay.getGraphics().clear();
        for(Map.Entry<String, NavigateWaypoint> entry : waypoints.entrySet()) {

            SimpleMarkerSceneSymbol symbol = new SimpleMarkerSceneSymbol(SimpleMarkerSceneSymbol.Style.CYLINDER, 0xFF0000FF, 1, 1, 1, SceneSymbol.AnchorPosition.CENTER);
            TextSymbol nameText = new TextSymbol(10, entry.getKey(), 0xFFFFFFFF, TextSymbol.HorizontalAlignment.LEFT, TextSymbol.VerticalAlignment.MIDDLE);
            nameText.setOffsetX(25);
            Graphic waypointGraphic = new Graphic(entry.getValue().toPoint(), new CompositeSymbol(Arrays.asList(symbol, nameText)));
            waypointOverlay.getGraphics().add(waypointGraphic);
        }

        updateFixtures();
    }

    private void updateBoundaryOverlay() {
        rtlBoundary.setItems(FXCollections.observableList(boundaries.keySet().stream().sorted().collect(Collectors.toList())));

        boundaryOverlay.getGraphics().clear();

        lawnmower.setDisable(!(!drones.getSelectionModel().isEmpty() && !boundaries.isEmpty()));
        random.setDisable(!(!drones.getSelectionModel().isEmpty() && !boundaries.isEmpty()));

        int color = 0;
        for(Map.Entry<String, List<Waypoint>> entry : boundaries.entrySet()) {
            List<Waypoint> boundary = entry.getValue();

            PointCollection polygonPoints = new PointCollection(boundary.stream().map(Waypoint::toPoint).collect(Collectors.toList()));
            SimpleFillSymbol polygonSymbol = new SimpleFillSymbol(SimpleFillSymbol.Style.SOLID, 0x000000, new SimpleLineSymbol(SimpleLineSymbol.Style.SOLID, COLORS[color], .3f));

            TextSymbol nameText = new TextSymbol(10, entry.getKey(), 0xFFFFFFFF, TextSymbol.HorizontalAlignment.LEFT, TextSymbol.VerticalAlignment.MIDDLE);
            nameText.setOffsetX(25);

            boundaryOverlay.getGraphics().add(new Graphic(new Polygon(polygonPoints), polygonSymbol));
            boundaryOverlay.getGraphics().add(new Graphic(boundary.get(0).toPoint(), nameText));

            color = (color + 1) % COLORS.length;
        }

        updateFixtures();
    }

    private void updateFixtures() {
        fixtureList.clear();

        for(Map.Entry<String, NavigateWaypoint> entry : waypoints.entrySet()) {
            final String name = entry.getKey();
            fixtureList.add(new WaypointFixture(entry.getKey(), entry.getValue(), () -> {waypoints.remove(name); updateWaypointOverlay();}));
        }

        for(Map.Entry<String, List<Waypoint>> entry : boundaries.entrySet()) {
            final String name = entry.getKey();
            fixtureList.add(new BoundaryFixture(entry.getKey(), entry.getValue(), () -> {boundaries.remove(name); updateBoundaryOverlay();}));
        }
    }

    private void updatePlumes() {
        plumeList.clear();

        for (Map.Entry<String, Plume> plumeEntry : plumes.entrySet()) {
            final String name = plumeEntry.getKey();
            plumeList.add(new NamedPlume(name, plumeEntry.getValue(), () -> {plumes.remove(name); updatePlumes();}));
        }
    }

    private void draw(List<Point> path) {
        System.out.println("Drawing...");
        pathOverlay.getGraphics().clear();

        PolylineBuilder lineBuilder = new PolylineBuilder(SpatialReferences.getWgs84());
        for(Point point : path) {
            lineBuilder.addPoint(point);
        }

        SimpleLineSymbol lineSymbol = new SimpleLineSymbol(SimpleLineSymbol.Style.DASH, 0xFF800080, 1);
        Graphic graphic = new Graphic(lineBuilder.toGeometry(), lineSymbol);

        pathOverlay.getGraphics().add(graphic);
    }

    private void centerDrone(Drone drone) {
         drone.getLatestPosition()
                 .observeOn(JavaFxScheduler.platform())
                 .map(ViewpointUtil::create)
                 .subscribe(camera -> sceneView.setViewpointAsync(camera));
    }

    private void deleteDrone(Drone name) {
        droneList.remove(name);
        name.shutdown();
        log("Removed " + name);
    }

    private void addDrone(String name) {
        if(!exists(name)) {
            Drone drone = new Drone(bridge, name);
            drone.init();

            drone.getLog()
                    .observeOn(JavaFxScheduler.platform())
                    .subscribe(message -> log(name + ": " + message));

            drone.getPositions()
                    .observeOn(JavaFxScheduler.platform())
                    .subscribe(new Observer<Drone.LatLonRelativeAltitude>() {
                        private Graphic droneGraphic;
                        private Graphic droneShadowGraphic;
                        private SimpleMarkerSceneSymbol symbol;
                        private Disposable updateColorDisposable;

                        @Override
                        public void onSubscribe(Disposable d) {
                        }

                        @Override
                        public void onNext(Drone.LatLonRelativeAltitude navSatFix) {
                            Point point = new Point(navSatFix.getLongitude(), navSatFix.getLatitude(), navSatFix.getRelativeAltitude());
                            if (droneGraphic == null) {
                                symbol = new SimpleMarkerSceneSymbol(SimpleMarkerSceneSymbol.Style.CYLINDER, 0xFFFF0000, 1, 1, 1, SceneSymbol.AnchorPosition.CENTER);
                                TextSymbol nameText = new TextSymbol(10, name, 0xFFFFFFFF, TextSymbol.HorizontalAlignment.LEFT, TextSymbol.VerticalAlignment.MIDDLE);
                                nameText.setOffsetX(25);
                                droneGraphic = new Graphic(point, new CompositeSymbol(Arrays.asList(symbol, nameText)));
                                droneOverlay.getGraphics().add(droneGraphic);
                                SimpleMarkerSymbol shadowSymbol = new SimpleMarkerSymbol(SimpleMarkerSymbol.Style.CIRCLE, 0x99000000, 2.5f);
                                droneShadowGraphic = new Graphic(point, shadowSymbol);
                                droneShadowOverlay.getGraphics().add(droneShadowGraphic);
                            } else {
                                droneGraphic.setGeometry(point);
                                droneShadowGraphic.setGeometry(point);
                                symbol.setColor(0xFFFF0000);
                            }
                            if(updateColorDisposable != null) {
                                updateColorDisposable.dispose();
                            }
                            updateColorDisposable = Observable.timer(10, TimeUnit.SECONDS)
                                    .subscribe(time -> symbol.setColor(0xFFD3D3D3));
                        }

                        @Override
                        public void onError(Throwable e) {

                        }

                        @Override
                        public void onComplete() {
                            if (droneGraphic != null) {
                                droneOverlay.getGraphics().remove(droneGraphic);
                                droneShadowOverlay.getGraphics().remove(droneShadowGraphic);
                            }
                        }
                    });

            droneList.add(drone);

            log("Added " + name);

            if(droneList.size() == 1 && !zoomedToFirst) {
                centerDrone(drone);
                zoomedToFirst = true;
            }
        }
    }

    private boolean exists(String name) {
        for(Drone drone : droneList) {
            if(name.equals(drone.getName())) {
                return true;
            }
        }
        return false;
    }

    private void log(String message) {
        logList.add(0, "[" + DATE_FORMAT.format(new Date()) + "]" + message);
    }

    void terminate() {
        if (sceneView != null) {
            sceneView.dispose();
        }
    }
}
