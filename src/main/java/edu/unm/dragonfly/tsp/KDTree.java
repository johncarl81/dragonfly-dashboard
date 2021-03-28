package edu.unm.dragonfly.tsp;


import edu.unm.dragonfly.ProjectedPoint;

/**
 * @author John Ericksen
 */
public class KDTree {

    private Node tree;

    public static class PointDistance {
        private final ProjectedPoint point;
        private final double distance;

        public PointDistance(ProjectedPoint point, double distance) {
            this.point = point;
            this.distance = distance;
        }

        public ProjectedPoint getPoint() {
            return point;
        }

        public double getDistance() {
            return distance;
        }
    }

    private static class Node {
        private final ProjectedPoint value;
        private final boolean orientation;
        private Node left = null;
        private Node right = null;
        private boolean deactivated = false;

        private Node(ProjectedPoint value) {
            this(value, false);
        }

        private Node(ProjectedPoint value, boolean orientation) {
            this.value = value;
            this.orientation = orientation;
        }

        public boolean comparison(ProjectedPoint point) {
            if(orientation) {
                return value.getY() < point.getY();
            } else {
                return value.getX() < point.getX();
            }
        }

        public void deactivate() {
            deactivated = true;
        }
    }

    public void remove(ProjectedPoint point) {
        remove(tree, point);
    }

    public void remove(Node parent, ProjectedPoint point) {
        if(parent != null) {
            if(parent.value == point) {
                parent.deactivate();
            } else {
                if(parent.comparison(point)) {
                    remove(parent.left, point);
                } else {
                    remove(parent.right, point);
                }
            }
        }
    }

    public void insert(ProjectedPoint point) {
        if(tree == null) {
            tree = new Node(point);
        }
        else {
            insert(tree, point);
        }
    }
    public void insert(Node parent, ProjectedPoint point){
        if(parent.comparison(point)) {
            if(parent.left == null) {
                parent.left = new Node(point, !parent.orientation);
            } else {
                insert(parent.left, point);
            }
        } else {
            if(parent.right == null) {
                parent.right = new Node(point, !parent.orientation);
            } else {
                insert(parent.right, point);
            }
        }
    }

    public PointDistance nearest(ProjectedPoint point) {
        return nearest(tree, point);
    }

    public double getAxisDistance(Node parent, ProjectedPoint point){
        if(parent.orientation) {
            return Math.abs(parent.value.getY() - point.getY());
        } else {
            return Math.abs(parent.value.getX() - point.getX());
        }
    }

    public PointDistance nearest(Node parent, ProjectedPoint point) {
        double parentDistance;
        if(!parent.deactivated) {
            parentDistance = point.distanceTo(parent.value);
        } else {
            parentDistance = Double.POSITIVE_INFINITY;
        }

        PointDistance nearestPoint = new PointDistance(parent.value, parentDistance);
        if(parent.comparison(point)) {
            if(parent.left != null) {
                nearestPoint = nearest(parent.left, point);
            }
        } else {
            if(parent.right != null) {
                nearestPoint = nearest(parent.right, point);
            }
        }

        // If we're still within a distance of the split, search the other split as well.
        if(nearestPoint.distance > getAxisDistance(parent, point)) {

            if(!parent.comparison(point)) {
                if(parent.left != null) {
                    PointDistance alternate = nearest(parent.left, point);
                    if(alternate.distance < nearestPoint.distance) {
                        nearestPoint = alternate;
                    }
                }
            } else {
                if(parent.right != null) {
                    PointDistance alternate = nearest(parent.right, point);
                    if(alternate.distance < nearestPoint.distance) {
                        nearestPoint = alternate;
                    }
                }
            }
        }

        if(nearestPoint.distance > parentDistance) {
            nearestPoint = new PointDistance(parent.value, parentDistance);
        }

        return nearestPoint;
    }
}
