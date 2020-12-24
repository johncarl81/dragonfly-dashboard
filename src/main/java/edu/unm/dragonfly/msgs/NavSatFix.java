package edu.unm.dragonfly.msgs;

import ros.msgs.std_msgs.Header;

/**
 * @author John Ericksen
 */
public class NavSatFix {
    public Header header;
    public NavSatStatus status;
    public double latitude;
    public double longitude;
    public double altitude;
    public double[] position_covariance;
    public int position_covariance_type;
}
