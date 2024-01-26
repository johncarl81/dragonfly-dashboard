package edu.unm.dragonfly.mission;

import edu.unm.dragonfly.mission.step.*;
import javafx.beans.value.*;
import javafx.collections.*;
import javafx.scene.control.*;
import javafx.scene.layout.*;

import java.util.*;
import java.util.stream.*;

/**
 * @author John Ericksen
 */
public class MissionStepDialogFactory {

    public interface CreateMissionStep {
        void call(MissionStep step);
    }

    public static void create(CreateMissionStep callback, List<String> drones, List<String> waypoints, Map<String, List<Waypoint>> boundaries) {

        Map<MissionStepType, MissionStepCreator> creators = new EnumMap<>(MissionStepType.class){{
            put(MissionStepType.TAKEOFF, new TakeoffCreator(drones));
            put(MissionStepType.SLEEP, new SleepCreator(drones));
            put(MissionStepType.LAND, new LandCreator(drones));
            put(MissionStepType.GOTO_WAYPOINT, new GotoCreator(drones, waypoints));
            put(MissionStepType.SEMAPHORE, new SemaphoreCreator(drones));
            put(MissionStepType.RTL, new RTLCreator(drones));
            put(MissionStepType.DDSA, new DDSACreator(drones, waypoints));
            put(MissionStepType.LAWNMOWER, new LawnmowerCreator(drones, boundaries.keySet().stream().sorted().collect(Collectors.toList())));
            put(MissionStepType.NAVIGATION, new RandomPointCreator(drones, boundaries));
            put(MissionStepType.FLOCK, new FlockCreator(drones));
            put(MissionStepType.GRADIENT, new GradientCreator(drones));
            put(MissionStepType.CALIBRATION, new CalibrationCreator(drones));
            put(MissionStepType.CURTAIN, new CurtainCreator(drones, waypoints));
            put(MissionStepType.PUMP, new PumpCreator(drones));
            put(MissionStepType.SKETCH, new SketchCreator(drones));
            put(MissionStepType.VERTICAL_TRANSECT, new VerticalTransectCreator(drones, waypoints));
        }};

        Dialog<ButtonType> dialog = new Dialog<>();
        dialog.setTitle("Mission Step");
        dialog.setResizable(true);

        GridPane grid = new GridPane();
        ColumnConstraints constraints1 = new ColumnConstraints(100);
        ColumnConstraints constraints2 = new ColumnConstraints();
        constraints2.setHgrow(Priority.ALWAYS);
        grid.getColumnConstraints().addAll(constraints1, constraints2);

        ComboBox<MissionStepType> typeSelection = new ComboBox<>(FXCollections.observableArrayList(MissionStepType.values()));


        grid.add(new Label("Type:"), 0, 0);
        grid.add(typeSelection, 1, 0);

        typeSelection.valueProperty().addListener(new ChangeListener<MissionStepType>() {
            @Override
            public void changed(ObservableValue<? extends MissionStepType> observable, MissionStepType oldValue, MissionStepType newValue) {
                grid.getChildren().remove(2, grid.getChildren().size());
                if(creators.containsKey(newValue)) {
                    creators.get(newValue).create(grid);
                }
                dialog.getDialogPane().getScene().getWindow().sizeToScene();
            }
        });

        dialog.getDialogPane().setContent(grid);
        dialog.getDialogPane().getButtonTypes().add(ButtonType.CANCEL);
        dialog.getDialogPane().getButtonTypes().add(ButtonType.OK);

        Optional<ButtonType> result = dialog.showAndWait();

        if(result.isPresent() && result.get() == ButtonType.OK) {
            if(creators.containsKey(typeSelection.getValue())) {
                callback.call(creators.get(typeSelection.getValue()).build());
            }
        }
    }
}
