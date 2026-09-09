import java.math.BigInteger;
import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Deque;
import java.util.HashMap;
import java.util.HashSet;
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

    public List<Route> findBestRoutes(double[] pi, int maxDrones, double dualTruck, double dualDrone, Set<String> usedRoutes) {
        List<Route> newRoutes = new ArrayList<>();

        int oriNgSize = Constant.LABEL_MAX_NG_SIZE;
        int maxD = maxDrones + 1;
        

        int totalBudget = 210;
        int remainingBudget = totalBudget;

        for(int d = 0; d < maxD && remainingBudget > 0; d++) {
            
            Constant.LABEL_MAX_NG_SIZE = Math.min(3, oriNgSize);

            int stage1Limit = Math.min(remainingBudget, 30);
            List<Route> stage1 = solveRCSPLimit(d, pi, dualTruck, dualDrone, stage1Limit, 10_000 , usedRoutes);
            remainingBudget -= stage1.size();
            newRoutes.addAll(stage1);
            

            Constant.LABEL_MAX_NG_SIZE = oriNgSize;

            if(remainingBudget <= 0) break;

            int stage2Limit = Math.min(remainingBudget, 30);
            List<Route> stage2 = solveRCSPLimit(d, pi, dualTruck, dualDrone, stage2Limit, 50_000, usedRoutes);
            remainingBudget -= stage2.size();
            newRoutes.addAll(stage2);

            if(remainingBudget <= 0) break;

            int stage3Limit = Math.min(remainingBudget, 150);
            List<Route> stage3 = solveRCSPLimit(d, pi, dualTruck, dualDrone, stage3Limit, Integer.MAX_VALUE, usedRoutes);
            remainingBudget -= stage3.size();
            newRoutes.addAll(stage3);

        }

        // System.out.println(newRoutes.size());

        return newRoutes;
    }

    private List<Route> solveRCSPLimit(int d, double[] pi, double dualTruck, double dualDrone, int maxColumn, int maxLabel, Set<String> usedRoutes) {
        List<Route> newRoutes = new ArrayList<>();

        RCSPGraph graph = buildGraph(d, pi);
        int src = Constant.TOTAL_CUSTOMER + 1; // 0'
        Label srcLabel = new Label(src);
        srcLabel.time = srcLabel.cost = 0;

        TreeMap<Double, List<Label>> buckets = new TreeMap<>();
        addLabelToBucket(buckets, srcLabel, 0);

        int sinkCount = 0;
        int countProcessedLabel = 0;
        while(!buckets.isEmpty() && countProcessedLabel < maxLabel) {
            List<Label> labels = buckets.pollFirstEntry().getValue();
            
            countProcessedLabel += labels.size();
            for(Label label : labels) {
                if(newRoutes.size() >= maxColumn) break;

                if(label.node == 0) { // reach sink label

                    sinkCount++;
                    // System.out.println("Sink reached: " + sinkCount + ", cost = " + label.cost);

                    if(label.cost < 0) {
                        Route route = reconstructRoute(label);
                        route.reducedCost = label.cost - dualTruck - d*dualDrone;

                        String sig = route.getSignature();
                        if (route.reducedCost < 0 && !usedRoutes.contains(sig)) {
                            newRoutes.add(route);
                            usedRoutes.add(sig);
                            // System.out.println(" V  ADDED: rc=" + route.reducedCost + ", sig=" + sig);
                        } else {
                            // System.out.println(" X  SKIPPED: rc=" + route.reducedCost + ", inUsed=" + usedRoutes.contains(sig) + ", sig=" + sig);
                        }
                            
                    }

                    continue; // reach sink, continue finding routes
                }

                for(RCSPARC arc : graph.adjacencyList.get(label.node)) {
                    if (arc.customerServed != null) {
                        boolean alreadyServed = false;
                        for (int cust : arc.customerServed) {
                            if (label.customerServed.contains(cust)) {
                                alreadyServed = true;
                                break;
                            }
                        }
                        if (alreadyServed) continue;
                    }

                    double newTime = label.time + arc.time;
                    if(newTime > Constant.TRUCK_MAX_SHIFT_TIME) continue;

                    Label newLabel = new Label(arc.dst);
                    newLabel.time = newTime;
                    newLabel.cost = label.cost + arc.cost;
                    newLabel.predecessor = label;
                    newLabel.arc = arc;

                    newLabel.ngSet.addAll(label.ngSet);
                    newLabel.ngDeque.addAll(label.ngDeque);
                    newLabel.addToNg(label.node);

                    newLabel.customerServed.addAll(label.customerServed);
                    if(arc.customerServed != null) newLabel.customerServed.addAll(arc.customerServed);

                    if(!isDominated(newLabel, buckets)) {
                        double bucketKey = Math.round(newLabel.cost * 10000.0) / 10000.0;
                        addLabelToBucket(buckets, newLabel, bucketKey); 
                    }

                }
            }

            if(newRoutes.size() >= maxColumn) break;

        }

        // System.out.println(newRoutes.size());

        return newRoutes;
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

                // if (i == 0) {
                //     System.out.println("Arc from 0' to " + j + ": drivingTime=" + drivingTime + ", reducedCost=" + reducedCost);
                // }
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

                if(arc.droneSchedule != null)
                    droneScheduleMap.put(arc.src, arc.droneSchedule);

                else if(arc.dst != 0)
                    sequence.add(0, arc.dst);
                    
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

                if(label.cost <= newLabel.cost + Constant.EPSILON
                && label.time <= newLabel.time + Constant.EPSILON
                && label.ngSet.containsAll(newLabel.ngSet)
                && label.customerServed.containsAll(newLabel.customerServed)
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
        Deque<Integer> ngDeque = new ArrayDeque<>();
        Set<Integer> customerServed = new HashSet<>();
        Label predecessor;
        RCSPARC arc;

        Label(int node) {
            this.node = node;
        }

        void addToNg(int node) {
            if (ngSet.contains(node)) return;
            ngDeque.addLast(node);
            ngSet.add(node);
            while (ngDeque.size() > Constant.LABEL_MAX_NG_SIZE) {
                int oldest = ngDeque.pollFirst();
                ngSet.remove(oldest);
            }
        }
    }

}
