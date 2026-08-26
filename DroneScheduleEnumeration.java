import java.math.BigInteger;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;

public class DroneScheduleEnumeration {
    private final double[][] dist = VRPInstance.distMatrix;
    private final List<List<Integer>> nodeNeighbourhood = new ArrayList<>();
    private final List<List<List<Integer>>> subsetsPool = new ArrayList<>();

    public static List<List<Map<BigInteger, ParetoFront>>> paretoMap = new ArrayList<>();

    public DroneScheduleEnumeration() {}

    private boolean isNeighbour(int src, int dst) {

        if (src == dst)
            return false;

        int demand = VRPInstance.nodes.get(dst).demand;
        if (demand > Constant.DRONE_PAYLOAD)
            return false;

        double distance = dist[src][dst];

        double W = Constant.DRONE_WEIGHT;
        double m = Constant.DRONE_BATTERY_WEIGHT;
        double q = demand;

        double g = Constant.G_FORCE;
        double p = Constant.AIR_DENSITY;
        double S = Constant.DRONE_SPINNING_BLADE_AREA;
        int h = Constant.DRONE_BLADE_NUMBER;

        double B_c = Constant.DRONE_BATTERY_CAPACITY; // W*h

        double back = Math.sqrt(g * g * g / (2 * p * S * h));
        double energyGo = Math.pow(W + m + q, 1.5) * back; // W
        double energyBack = Math.pow(W + m, 1.5) * back; // W

        double totalTime = distance / Constant.DRONE_SPEED; // h

        double totalEnergy = totalTime * (energyGo + energyBack); // W*h

        return totalEnergy <= B_c;
    }

    private void buildNodeNeighbourhood() {
        nodeNeighbourhood.add(Collections.emptyList()); // depot has no neighbour

        for (int src = 1; src <= Constant.TOTAL_CUSTOMER; src++) {
            List<Integer> neighbours = new ArrayList<>();
            for (int dst = 1; dst <= Constant.TOTAL_CUSTOMER; dst++) {
                if (isNeighbour(src, dst))
                    neighbours.add(dst);
            }

            final int from = src;
            neighbours = neighbours.stream()
                    .sorted(Comparator.comparingDouble(i -> dist[from][i]))
                    .limit(Constant.MAX_NEIGHBOURS_PER_NEIGHBOURHOOD)
                    .toList();

            nodeNeighbourhood.add(neighbours);
        }
    }

    private List<List<Integer>> getSubsets(int d) {
        int size = subsetsPool.size();

        if (d < size && subsetsPool.get(d) != null)
            return subsetsPool.get(d);

        for (; size <= d; size++) {
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
        if (d1.src != d2.src)
            return null;

        if (!d1.customerServed.and(d2.customerServed).equals(BigInteger.ZERO))
            return null; // schedule_1.cusServed ^ schedule_2.cusServed != 0 -> same customer(s) served by both schedule 

        BigInteger newCustomerServed = d1.customerServed.or(d2.customerServed);

        List<List<Integer>> newSequences = new ArrayList<>();

        for (var d : d1.sequences)
            newSequences.add(new ArrayList<>(d));
        for (var d : d2.sequences)
            newSequences.add(new ArrayList<>(d));

        double newMakespan = Math.max(d1.makespan, d2.makespan);
        double newMaxStartingTime = Math.min(d1.maxStartingTime, d2.maxStartingTime);

        return new DroneSchedule(d1.src, newSequences, newMakespan, newMaxStartingTime, newCustomerServed);

    }

    public void solve() {
        paretoMap.clear();

        buildNodeNeighbourhood();

        paretoMap.add(Collections.emptyList()); // depot

        int nodeIterTime = Math.min(Constant.TOTAL_CUSTOMER, Config.MAX_NODE_LOOP);

        for (int node = 1; node <= nodeIterTime; node++) {
            System.out.println("Node " + node);

            List<Integer> neighbours = nodeNeighbourhood.get(node);

            if (neighbours == null || neighbours.isEmpty())
                continue;

            List<List<Integer>> subsets = getSubsets(neighbours.size());

            List<Map<BigInteger, ParetoFront>> S_d = new ArrayList<>();
            S_d.add(Collections.emptyMap()); // drone num = 0

            System.out.println(node + " 1");

            Map<BigInteger, ParetoFront> S_1 = new HashMap<>();
            for (List<Integer> subset : subsets) {
                List<Integer> sequence = new ArrayList<>();
                for (int i : subset)
                    sequence.add(neighbours.get(i));

                Collections.sort(sequence, Comparator.comparingDouble(i -> VRPInstance.nodes.get(i).tw_b));

                BigInteger customerServed_1 = BigInteger.ZERO;
                for (int cus : sequence) {
                    customerServed_1 = customerServed_1.setBit(cus);
                }

                List<List<Integer>> newSequences = new ArrayList<>();
                newSequences.add(sequence);

                ParetoFront pf = new ParetoFront();
                DroneSchedule newDroneSchedule;
                try {
                    newDroneSchedule = new DroneSchedule(node, newSequences);
                    pf.tryAddSchedule(newDroneSchedule);
                } catch(IllegalArgumentException ex) {}

                if(!pf.nonDominatedSchedules.isEmpty()) {
                    S_1.put(customerServed_1, pf);
                }
            }

            S_d.add(S_1);

            for (int numDrone = 2; numDrone <= Constant.MAX_DRONE_PER_VEHICLE; numDrone++) {
                System.out.println(node + " " + numDrone);
                Map<BigInteger, ParetoFront> S_numDrone = new HashMap<>();

                Set<Integer> asdfjkl= new HashSet<>();

                for(int numDrone_a = 1; numDrone_a <= numDrone/2; numDrone_a++) {
                    int numDrone_b = numDrone - numDrone_a;

                    for (var set_numDrone_a : S_d.get(numDrone_a).entrySet()) {
                        for (var set_numDrone_b : S_d.get(numDrone_b).entrySet()) {

                            // if(numDrone_a == numDrone_b
                            // && set_numDrone_a.getKey().compareTo(set_numDrone_b.getKey()) > 0
                            // ) continue;
                            
                            for(DroneSchedule schedule_numDrone_a : set_numDrone_a.getValue().nonDominatedSchedules) {
                                for(DroneSchedule schedule_numDrone_b : set_numDrone_b.getValue().nonDominatedSchedules) {

                                    DroneSchedule combined = combine(schedule_numDrone_a, schedule_numDrone_b);

                                    if (combined == null || asdfjkl.contains(hashCode(combined)))
                                        continue;
                                    asdfjkl.add(hashCode(combined));

                                    BigInteger combined_customerServed = schedule_numDrone_a.customerServed
                                                                     .or(schedule_numDrone_b.customerServed);

                                    if (combined_customerServed.bitCount() > Constant.MAX_NEIGHBOURS_PER_NEIGHBOURHOOD)
                                        continue;

                                    ParetoFront pf = S_numDrone.getOrDefault(combined_customerServed, new ParetoFront());
                                    pf.tryAddSchedule(combined);
                                    S_numDrone.put(combined_customerServed, pf);

                                }
                            }
                        }
                    }
                }

                S_d.add(S_numDrone);
            }

            paretoMap.add(S_d);

        }
    }

    private int hashCode(DroneSchedule schedule) {
        int hash = 0;
        for (List<Integer> sub : schedule.sequences) {
            hash += sub.hashCode();   // commutative: order doesn't matter
        }
        return hash;
    }

}
