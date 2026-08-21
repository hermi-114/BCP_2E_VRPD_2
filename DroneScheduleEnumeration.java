import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

public class DroneScheduleEnumeration {
    public static double[][] dist = VRPInstance.distMatrix;
    public static List<List<Integer>> nodeNeibourhood;
    public static List<List<Integer>> subsets;
    
    private boolean isNeighbour(int src, int dst) {

        if(src == dst) return false;

        Node dstNode = VRPInstance.nodes.get(dst);

        int demand = dstNode.demand;
        if(demand > Constant.DRONE_PAYLOAD) return false;

        double distance = dist[src][dst];

        double W = Constant.DRONE_WEIGHT;
        double m = Constant.DRONE_BATTERY_WEIGHT;
        double q = demand;

        double g = Constant.G_FORCE;
        double p = Constant.AIR_DENSITY;
        double S = Constant.DRONE_SPINNING_BLADE_AREA;
        int h = Constant.DRONE_BLADE_NUMBER;

        double B_c = Constant.DRONE_BATTERY_CAPACITY; // W*h

        double back =  Math.sqrt(g*g*g / (2*p*S*h));
        double energyGo =   Math.pow(W + m + q, 1.5) * back; // W
        double energyBack = Math.pow(W + m    , 1.5) * back; // W

        double totalTime = distance / Constant.DRONE_SPEED; // h

        double totalEnergy = totalTime * (energyGo + energyBack); // W*h

        return B_c - totalEnergy > 0;
    }

    private void buildNodeNeighbourhood() {
        nodeNeibourhood.add(Collections.emptyList()); // depot has no neighbour

        for(int src = 1; src <= Constant.TOTAL_CUSTOMER; src++) {
            List<Integer> neighbour = new ArrayList<>();
            for(int dst = 1; dst <= Constant.TOTAL_CUSTOMER; dst++) {
                if(isNeighbour(src, dst)) neighbour.add(dst);
            }
            nodeNeibourhood.add(neighbour);
        }
    }

    public void solve() {
        buildNodeNeighbourhood();
    }

}
