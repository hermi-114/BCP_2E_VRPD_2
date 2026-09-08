import java.math.BigInteger;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Iterator;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;
import java.util.Set;
import java.util.TreeMap;

public class PricingProblem {
    
    // Build graph G^d
    // Run Bucket Graph Labeling Algorithm
    // Arc costs = PhysicalCost - dualVariables[customer] (if customer is served)
    // Find path with most negative reduced cost

    CuttingPlanes cuttingPlanes;

    public PricingProblem(CuttingPlanes cuttingPlanes) {
        this.cuttingPlanes = cuttingPlanes;
    }
    
    public Route findBestRoute(double[] pi, int maxDrones, double dualTruck, double dualDrone) {

        Route bestRoute = null;
        double bestReducedCost = Double.POSITIVE_INFINITY;

        for(int d = 0; d <= maxDrones; d++) {

            Route candidate = solveRCSP(d, pi, dualTruck, dualDrone);

            if(candidate == null) continue;

            // if(ColumnGeneration.existedSequences.contains(candidate.getSequence())) continue;

            double reducedCost = candidate.reducedCost;
            if(reducedCost < bestReducedCost) {
                bestRoute = candidate;
                bestReducedCost = reducedCost;
            }
        }

        return bestRoute;
    }

    private Route solveRCSP(int d, double[] pi, double dualTruck, double dualDrone) {

        RCSPGraph graph = buildGraph(d, pi);

        int src = Constant.TOTAL_CUSTOMER + 1; // 0'
        Label srcLabel = new Label(src);
        srcLabel.cost = 0;
        srcLabel.time = 0;

        TreeMap<Double, List<Label>> buckets = new TreeMap<>();
        addLabelToBucket(buckets, srcLabel, 0);

        Label bestSinkLabel = null;
        double bestSinkCost = Double.POSITIVE_INFINITY;

        while(!buckets.isEmpty()) {
            Entry<Double, List<Label>> entry = buckets.pollFirstEntry();
            List<Label> labels = entry.getValue();

            for(Label label : labels) {
                if(label.cost - bestSinkCost >= -Constant.EPSILON) continue;

                if(label.node == 0) {
                    if(label.cost < bestSinkCost) {
                        bestSinkCost = label.cost;
                        bestSinkLabel = label;
                    } continue;
                }

                for(RCSPARC arc : graph.adjacencyList.get(label.node)) {

                    // check if duplicate customer
                    if (arc.customerServed != null) {
                        boolean existed = false;
                        for (int cust : arc.customerServed) {
                            if (label.customerServed.contains(cust)) {
                                existed = true;
                                break;
                            }
                        }
                        if (existed) continue;
                    }

                    double newTime = label.time + arc.time;

                    if(newTime > Constant.TRUCK_MAX_SHIFT_TIME) continue;

                    Label newLabel = new Label(arc.dst);
                    newLabel.cost = label.cost + arc.cost;
                    newLabel.time = newTime;
                    newLabel.predecessor = label;
                    newLabel.arc = arc;

                    newLabel.ngSet.addAll(label.ngSet);
                    newLabel.ngSet.add(label.node);
                    while(newLabel.ngSet.size() > Constant.LABEL_MAX_NG_SIZE) {
                        Iterator<Integer> it = newLabel.ngSet.iterator();
                        if(it.hasNext()) {
                            it.next();
                            it.remove();
                        }
                    }

                    newLabel.customerServed.addAll(label.customerServed);
                    if(arc.customerServed != null) newLabel.customerServed.addAll(arc.customerServed);

                    if(isDominated(newLabel, buckets)) continue;

                    double bucketKey = Math.round(newLabel.cost * 10000.0) / 10000.0;
                    addLabelToBucket(buckets, newLabel, bucketKey);
                }
            }

            
        }

        if(bestSinkLabel != null && bestSinkLabel.cost < 0) {
            Route route = reconstructRoute(bestSinkLabel);
            route.reducedCost = bestSinkLabel.cost - dualTruck - d*dualDrone;
            return route;
        }

        return null;
    }

    private RCSPGraph buildGraph(int d, double[] pi) {
        RCSPGraph graph = new RCSPGraph();
        int totalNode = Constant.TOTAL_CUSTOMER + 1;
        int numNode = 2 * totalNode;

        for(int i = 0; i < numNode; i++) {
            graph.adjacencyList.add(new ArrayList<>());
        }


        // ====== INITIALIZE TRUCK ARC: from i' to j ====== 
        for(int i = 0; i < totalNode; i++) {
            int from = totalNode + i;
            for(int j = 0; j < totalNode; j++) {
                if(i == j) continue;

                double drivingTime = VRPInstance.distMatrix[i][j] / Constant.TRUCK_SPEED;
                double reducedCost = drivingTime; // initial trucks do not have duals

                List<Integer> customerServed = null;
                if(j != 0) {
                    reducedCost -= pi[j-1];
                    reducedCost += cuttingPlanes.getReducedCostPenaltyForTruckArc(i, j, d);
                    customerServed = List.of(j);
                }
                
                RCSPARC arc = new RCSPARC(from, j, reducedCost, drivingTime, customerServed);
                graph.adjacencyList.get(from).add(arc);
            }
        }


        // ====== INITIALIZE DRONE ARC: from i to i' ====== 
        for(int i = 1; i <= Constant.TOTAL_CUSTOMER; i++) {
            int to = i + totalNode;
            double servingTime = VRPInstance.nodes.get(i).servingTime;

            DroneSchedule emptySchedule = new DroneSchedule(i);
            RCSPARC emptyArc = new RCSPARC(i, to, servingTime, servingTime, new ArrayList<>(), emptySchedule);
            graph.adjacencyList.get(i).add(emptyArc);

            if(d == 0) continue;
            if(d >= DroneScheduleEnumeration.paretoMap.get(i).size()) continue;

            Set<Entry<BigInteger, ParetoFront>> schedules = DroneScheduleEnumeration.paretoMap.get(i).get(d).entrySet();

            for(var entry : schedules) {
                for(DroneSchedule schedule : entry.getValue().nonDominatedSchedules) {
                    double physicalCost = Math.max(servingTime, schedule.makespan);

                    double reducedCost = physicalCost;
                    for(int cust : schedule.customerServed) {
                        reducedCost -= pi[cust-1]; // pi 0-based
                    }

                    reducedCost += cuttingPlanes.getReducedCostPenaltyForDroneArc(i, schedule, d);

                    schedule.reducedCost = reducedCost;
                    double time = schedule.makespan;
                    List<Integer> servedCustomers = new ArrayList<>(schedule.customerServed);

                    RCSPARC arc = new RCSPARC(i, to, reducedCost, time, servedCustomers, schedule);
                    graph.adjacencyList.get(i).add(arc);
                }
            }

        }

        return graph;
    }

    private Route reconstructRoute(Label sinkLabel) {
        List<Integer> sequence = new ArrayList<>();
        Map<Integer, DroneSchedule> droneScheduleMap = new HashMap<>();
        
        Label current = sinkLabel;
        while(current.predecessor != null) {
            RCSPARC arc = current.arc;
            if(arc != null) {
                if(arc.droneSchedule != null) {
                    droneScheduleMap.put(arc.src, arc.droneSchedule);
                } else if(arc.dst != 0) {
                    sequence.add(0, arc.dst);
                }
                
            }
            current = current.predecessor;
        }

        List<Node> routeSequence = new ArrayList<>();
        for(int cust : sequence) routeSequence.add(VRPInstance.nodes.get(cust));

        Route route = new Route(routeSequence, droneScheduleMap);
        route.reducedCost = sinkLabel.cost;

        return route;
    }

    private void addLabelToBucket(TreeMap<Double, List<Label>> buckets, Label label, double key) {
        buckets.computeIfAbsent(key, k -> new ArrayList<>()).add(label);
    }

    private boolean isDominated(Label newLabel, TreeMap<Double, List<Label>> buckets) {

        for(var entry : buckets.entrySet()) {
            if(entry.getKey() - newLabel.cost > Constant.EPSILON) break;

            for(Label label : entry.getValue()) {
                if(newLabel.node != label.node) continue;

                if (label.cost <= newLabel.cost + Constant.EPSILON &&
                    label.time <= newLabel.time + Constant.EPSILON &&
                    label.ngSet.containsAll(newLabel.ngSet)
                ) return true;
                    
            }
        }

        return false;
    }

    private static class RCSPGraph {
        List<List<RCSPARC>> adjacencyList = new ArrayList<>();
    }

    private static class RCSPARC {
        int src, dst;
        double cost;
        double time; // truck waiting/driving time
        List<Integer> customerServed;
        DroneSchedule droneSchedule;

        RCSPARC(int src, int dst, double cost, double time, List<Integer> customerServed) {
            this.src = src;
            this.dst = dst;
            this.cost = cost;
            this.time = time;
            this.customerServed = customerServed;
        }

        public RCSPARC(int src, int dst, double cost, double time, List<Integer> customers, DroneSchedule schedule) {
            this.src = src;
            this.dst = dst;
            this.cost = cost;
            this.time = time;
            this.customerServed = customers;
            this.droneSchedule = schedule;
        }

        
    }

    private static class Label {
        int node;
        double cost;
        double time;
        Set<Integer> ngSet = new HashSet<>();
        List<Integer> customerServed = new ArrayList<>();
        Label predecessor;
        RCSPARC arc;

        Label(int node) {
            this.node = node;
        }
    }

}
