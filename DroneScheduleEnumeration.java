import java.util.ArrayList;
import java.util.Collections;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

public class DroneScheduleEnumeration {
    private final double[][] dist = VRPInstance.distMatrix;
    private final List<List<Integer>> nodeNeighbourhood = new ArrayList<>();
    private final List<List<List<Integer>>> subsetsPool = new ArrayList<>();

    public static List<List<ParetoFront>> paretoMap = new ArrayList<>();
    
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

        return totalEnergy <= B_c;
    }

    private void buildNodeNeighbourhood() {
        nodeNeighbourhood.add(Collections.emptyList()); // depot has no neighbour

        for(int src = 1; src <= Constant.TOTAL_CUSTOMER; src++) {
            List<Integer> neighbour = new ArrayList<>();
            for(int dst = 1; dst <= Constant.TOTAL_CUSTOMER; dst++) {
                if(isNeighbour(src, dst)) neighbour.add(dst);
            }
            nodeNeighbourhood.add(neighbour);
        }
    }

    private List<List<Integer>> getSubsets(int d) {
        if(subsetsPool.get(d) != null) return subsetsPool.get(d);

        int start = subsetsPool.size();
        
        for(int size = start; size <= d; size++) {
            List<List<Integer>> subsets = new ArrayList<>();

            for (int i = 1; i < (1 << size); i++) {
                if (Integer.bitCount(i) > Constant.DRONE_MAX_STOP) {
                    continue;
                }

                List<Integer> set = new ArrayList<>();

                for (int j = 0; j < size; j++) {
                    if ((i & (1 << j)) != 0) {
                        set.add(j);
                    }
                }

                subsets.add(set);
            }

            subsetsPool.add(subsets);
        }

        return subsetsPool.get(d);
    }

    private DroneSchedule combine(DroneSchedule d1, DroneSchedule d2) { // try-catch when call
        if(d1.src != d2.src) return null;

        Set<Integer> newCustomerServed = new HashSet<>();
        newCustomerServed.addAll(d1.customerServed);
        for(var d2_customer : d2.customerServed) {
            if(newCustomerServed.contains(d2_customer)) return null; // customer was served by both d1 & d2
            newCustomerServed.add(d2_customer);
        }

        List<List<Integer>> newSequences = new ArrayList<>();
        
        for(var d : d1.sequences) newSequences.add(new ArrayList<>(d));
        for(var d : d2.sequences) newSequences.add(new ArrayList<>(d));

        double newMakespan = Math.max(d1.makespan, d2.makespan);
        double newMaxStartingTime = Math.min(d1.maxStartingTime, d2.maxStartingTime);

        return new DroneSchedule(d1.src, newSequences, newMakespan, newMaxStartingTime, newCustomerServed);

    }

    public void solve() {
        buildNodeNeighbourhood();

        paretoMap.add(Collections.emptyList()); // depot

        for(int node = 1; node <= Constant.TOTAL_CUSTOMER; node++) {
            List<Integer> neighbours = nodeNeighbourhood.get(node);
            List<List<Integer>> subsets = getSubsets(neighbours.size());
            List<List<Integer>> sequences = new ArrayList<>();

            for(var subset : subsets) {
                List<Integer> sequence = new ArrayList<>();
                for(int customerIdx : subset) sequence.add(neighbours.get(customerIdx));
                sequences.add(sequence);
            }

            List<ParetoFront> pareto_d = new ArrayList<>();
            pareto_d.add(null); // pareto for 0 drone

            ParetoFront p_1 = new ParetoFront();
            for(List<Integer> sequence : sequences) {
                List<List<Integer>> newSequences = new ArrayList<>();
                newSequences.add(sequence);
                p_1.tryAddSchedule(new DroneSchedule(node, newSequences));
            }

            pareto_d.add(p_1);

            for(int numDrone = 2; numDrone <= Constant.MAX_DRONE_PER_VEHICLE; numDrone++) {

                ParetoFront p_numDrone = new ParetoFront();

                for(int numDrone_a = 1; numDrone_a <= numDrone/2; numDrone_a++) {
                    int numDrone_b = numDrone - numDrone_a;

                    for(DroneSchedule s_a : pareto_d.get(numDrone_a).nonDominatedSchedules) 
                        for(DroneSchedule s_b : pareto_d.get(numDrone_b).nonDominatedSchedules) {
                            DroneSchedule conbined = combine(s_a, s_b);
                            p_numDrone.tryAddSchedule(conbined);
                        }
                }

                pareto_d.add(p_numDrone);
            }

            paretoMap.add(pareto_d);

        }
    }

}
