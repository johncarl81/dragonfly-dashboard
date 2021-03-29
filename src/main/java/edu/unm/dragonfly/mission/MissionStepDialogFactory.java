package edu.unm.dragonfly.mission;

import edu.unm.dragonfly.mission.step.MissionStep;
import edu.unm.dragonfly.mission.step.MissionStepType;
import javafx.beans.value.ChangeListener;
import javafx.beans.value.ObservableValue;
import javafx.collections.FXCollections;
import javafx.scene.control.ButtonType;
import javafx.scene.control.ComboBox;
import javafx.scene.control.Dialog;
import javafx.scene.control.Label;
import javafx.scene.layout.ColumnConstraints;
import javafx.scene.layout.GridPane;
import javafx.scene.layout.Priority;

import java.util.EnumMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.stream.Collectors;

/**
 * @author John Ericksen
 */
public class MissionStepDialogFactory {

    public interface CreateMissionStep {
        void call(MissionStep step);
    }

    public static void create(CreateMissionStep callback, List<String> drones, List<String> waypoints, Map<String, List<Waypoint>> boundaries) {

        Map<MissionStepType, MissionStepCreator> creators = new EnumMap<MissionStepType, MissionStepCreator>(MissionStepType.class){{
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
            put(MissionStepType.CURTAIN, new CurtainCreator(drones, waypoints));
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
