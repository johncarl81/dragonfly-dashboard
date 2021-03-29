package edu.unm.dragonfly.mission;

import javafx.scene.Node;
import javafx.scene.control.Label;
import javafx.scene.layout.GridPane;

/**
 * @author John Ericksen
 */
public class GridUtil {

    public interface GridBuilderInput {

        void create(GridBuilder gridBuilder);
    }

    public static class GridBuilder {

        private final GridPane grid;
        private int index = 0;

        private GridBuilder(GridPane grid) {
            this.grid = grid;
        }

        public GridBuilder add(GridBuilderInput delegate) {
            delegate.create(this);

            return this;
        }

        public GridBuilder add(Node node) {
            grid.add(node, 0, index, 1, index);

            index++;

            return this;
        }

        public GridBuilder add(String labelName, Node node) {
            grid.add(new Label(labelName), 0, index);
            grid.add(node, 1, index);

            index++;

            return this;
        }

        public GridBuilder increment() {
            index++;

            return this;
        }
    }

    public static GridBuilder builder(GridPane grid) {
        return new GridBuilder(grid);
    }
}
