import java.math.BigInteger;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Iterator;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;
import java.util.Set;


public class PricingProblem {
    
    public enum PricingStage {
        LIGHT,      // stage 1: one label per bucket
        HEURISTIC,  // stage 2: ignore ng-set and cut state
        EXACT       // stage 3: full dominance
    }
    // Build graph G^d
    // Run Bucket Graph Labeling Algorithm
    // Arc costs = PhysicalCost - dualVariables[customer] (if customer is served)
    // Find path with most negative reduced cost

    CuttingPlanes cuttingPlanes;
    List<RCSPGraph> graphs;

    @SuppressWarnings("unchecked")
    List<Label>[][][] buckets = (List<Label>[][][]) new List[Constant.TOTAL_CUSTOMER + 1][Config.SIZE_BUCKET][Config.SIZE_BUCKET]; // v, B_d, B_c
    
    Label[][][] dominatingLabels = new Label[Constant.TOTAL_CUSTOMER + 1][Config.SIZE_BUCKET][Config.SIZE_BUCKET];
    
    List<Route> newRoutes = new ArrayList<>();
    
    Set<String> signatures = new HashSet<>();

    public PricingProblem(CuttingPlanes cuttingPlanes) {
        this.cuttingPlanes = cuttingPlanes;
    }

    private static class RCSPGraph {
        List<List<RCSPArc>> adjacencyList;
    }

    // || ========================================================================= ||
    // || ====================== BUCKET GRAPH LABELLING =========================== ||
    // || ========================================================================= ||

    public void runThreeStagePricing(double[] pi, double dualTruck, double dualDrone) {

        // Stage 1 – light
        solveRCSP(pi, dualTruck, dualDrone, PricingStage.LIGHT);

        // Stage 2 – heuristic
        solveRCSP(pi, dualTruck, dualDrone, PricingStage.HEURISTIC);

        // Stage 3 – exact
        solveRCSP(pi, dualTruck, dualDrone, PricingStage.EXACT);
    }

    // =========================================================================
    // ---------------------------- MAIN LOOP ----------------------------------
    // =========================================================================
    private void solveRCSP(double[] pi, double dualTruck, double dualDrone, PricingStage stage) {

        buildGraphs(pi);
        int size = Config.SIZE_BUCKET;
        for(int i = 0; i <= Constant.TOTAL_CUSTOMER; i++) for(int j = 0; j < size; j++) for(int k = 0; k < size; k++) {
            buckets[i][j][k] = new ArrayList<>();
        }


        for(int d = 0; d <= Constant.MAX_DRONE_PER_VEHICLE; d++) {
        
            int src = Constant.TOTAL_CUSTOMER + 1; // 0'
            Label srcLabel = new Label(src);
            
            addLabelToBucket(srcLabel, stage);
            
            RCSPGraph graph = graphs.get(d);
            
            for(int idx = 1; idx <= Constant.TOTAL_CUSTOMER+1; idx++)
             for(int i = 0; i < Config.SIZE_BUCKET; i++)
              for(int j = 0; j < Config.SIZE_BUCKET; j++) {

                List<Label> labels = buckets[idx % (Constant.TOTAL_CUSTOMER + 1)][i][j];

                for(Label label : new ArrayList<>(labels)) {
                    if(label.node == 0) { // sink label, complete a route
                        if(label.reducedCost < 0) {
                            Route route = reconstructRoute(label);
                            
                            route.reducedCost = label.reducedCost - dualTruck - d*dualDrone;
                            String sig = route.getSignature();
                            
                            // check if valid: reduced cost, existance
                            // TODO
                            if (route.reducedCost >= Constant.EPSILON ||
                                signatures.contains(sig)) {
                                continue;
                            }

                            newRoutes.add(route);
                            signatures.add(sig);

                        }
                        continue;
                    }

                    for(RCSPArc arc : graph.adjacencyList.get(label.node % (1 + Constant.TOTAL_CUSTOMER))) {
                        
                        // check if valid customer served
                        if(!label.customerServed.and(arc.customerServed).equals(BigInteger.ZERO)) continue;

                        // check if new duration is valid
                        // TODO
                        double newDuration = label.duration + arc.duration;
                        double newCapacity = label.capacity + arc.capacity;
                        double newReducedCost = label.reducedCost + arc.reducedCost;
                        

                        Label newLabel = new Label(arc.dst);
                        newLabel.arc = arc;
                        newLabel.duration = newDuration;
                        newLabel.capacity = newCapacity;
                        newLabel.reducedCost = newReducedCost;
                        newLabel.predecessor = label;
                        newLabel.customerServed = label.customerServed.or(arc.customerServed);

                        addLabelToBucket(newLabel, stage);
      
                    }
                }
            }
            

        }

        

    }

    private void addLabelToBucket(Label newLabel, PricingStage stage) {
        int v = newLabel.node;
        Node cust = VRPInstance.nodes.get(newLabel.node);

        double tw_size = cust.tw_b - cust.tw_a;
        double capa_size = (double) Constant.TRUCK_PAYLOAD / Config.SIZE_BUCKET;

        int B_c = (int) Math.floor(newLabel.capacity / capa_size);
        int B_d = (int) Math.floor((newLabel.duration - cust.tw_a) / tw_size);

        if (B_d < 0 || B_d >= Config.SIZE_BUCKET
        ||  B_c < 0 || B_c >= Config.SIZE_BUCKET
            ) return;

        // is dominated in current bucket
        List<Label> bucket = buckets[v][B_d][B_c];

        switch (stage) {
            case LIGHT     -> addLabelLight(newLabel, bucket);
            case HEURISTIC -> addLabelWithDominance(newLabel, v, B_d, B_c, bucket, false);
            case EXACT     -> addLabelWithDominance(newLabel, v, B_d, B_c, bucket, true);
        }
    }

    private void addLabelLight(Label newLabel, List<Label> bucket) {
        if (bucket.isEmpty()) {
            bucket.add(newLabel);
            return;
        }

        Label best = bucket.get(0);
        if (newLabel.reducedCost < best.reducedCost - Constant.EPSILON) {
            bucket.clear();
            bucket.add(newLabel);
        }
    }

    private void addLabelWithDominance(Label newLabel, int v, int Bd, int Bc, List<Label> bucket, boolean checkNgAndCuts) {

        // 1. Check domination inside the same bucket
        Iterator<Label> it = bucket.iterator();
        while (it.hasNext()) {
            Label label = it.next();
            if (isDominates(label, newLabel, checkNgAndCuts)) return;
            if (isDominates(newLabel, label, checkNgAndCuts)) it.remove();
        }

        // 2. Check domination by component-wise smaller buckets
        for (int i = 0; i < Bd; i++) {
            for (int j = 0; j < Bc; j++) {
                for (Label label : buckets[v][i][j]) {
                    if (isDominates(label, newLabel, checkNgAndCuts)) return;
                }
            }
        }

        // 3. Add the new label
        bucket.add(newLabel);

        // 4. Update the best label for this bucket (if you use it)
        if (dominatingLabels[v][Bd][Bc] == null ||
            isDominates(newLabel, dominatingLabels[v][Bd][Bc], checkNgAndCuts)
        ) dominatingLabels[v][Bd][Bc] = newLabel;
        
    }

    private boolean isDominates(Label a, Label b, boolean checkNgAndCuts) {
        if (a == null || b == null) return false;
        if (a.node != b.node) return false;

        // Resource consumption: a must be <= b
        if (a.duration > b.duration + Constant.EPSILON) return false;
        if (a.capacity > b.capacity + Constant.EPSILON) return false;

        // ng-set (^) and cut state (6)
        if (checkNgAndCuts) {
            // ^ : a.customerServed must be a subset of b.customerServed
            if (!b.customerServed.and(a.customerServed).equals(a.customerServed)) {
                return false;
            }
            // 6 : if you have a cut-state field, check it here.
            // Example (simplified):
            // if (a.cutState > b.cutState) return false;
        }
        // Reduced cost: a must be <= b

        // add cut (6)
        // TODO

        return a.reducedCost <= b.reducedCost + Constant.EPSILON;
    }



    // =========================================================================
    // ---------------------------- BUILD GRAPH --------------------------------
    // =========================================================================

    private void buildGraphs(double[] pi) {
        graphs = new ArrayList<>();

        for(int numDrone = 0; numDrone <= Constant.MAX_DRONE_PER_VEHICLE; numDrone++) {
            graphs.add(buildGraph(numDrone, pi));
        }
    }

    private RCSPGraph buildGraph(int d, double[] pi) {
        RCSPGraph graph = new RCSPGraph();
        int size_V_plus = Constant.TOTAL_CUSTOMER + 1;
        int totalNode = 2 * size_V_plus;

        for(int i = 0; i < totalNode; i++) {
            graph.adjacencyList.add(new ArrayList<>());
        }


        // ================== INITIALIZE TRUCK ARC: from i' to j ================== 
        for(int i = 0; i < size_V_plus; i++) {
            int from = size_V_plus + i;
            for(int j = 0; j < size_V_plus; j++) {
                if(i == j) continue;

                double drivingTime = VRPInstance.distMatrix[i][j] / Constant.TRUCK_SPEED;
                double reducedCost = drivingTime - pi[j-1];
                double capacity = VRPInstance.nodes.get(j).demand;

                BigInteger customerServed = BigInteger.ZERO;
                if(j != 0) {
                    // reducedCost += cuttingPlanes.getReducedCostPenaltyForTruckArc(i, j, d);
                    customerServed = customerServed.setBit(j);
                }
                
                RCSPArc arc = new RCSPArc(from, j, drivingTime, capacity, reducedCost, customerServed, null);
                graph.adjacencyList.get(i).add(arc);

                // if (i == 0) {
                //     System.out.println("Arc from 0' to " + j + ": drivingTime=" + drivingTime + ", reducedCost=" + reducedCost);
                // }
            }
        }


        // ================== INITIALIZE DRONE ARC: from i to i' ================== 
        for(int i = 1; i <= Constant.TOTAL_CUSTOMER; i++) {
            int to = i + size_V_plus;
            double servingTime = VRPInstance.nodes.get(i).servingTime;

            DroneSchedule emptySchedule = new DroneSchedule(i);
            RCSPArc emptyArc = new RCSPArc(i, to, servingTime, 0, servingTime, BigInteger.ZERO, emptySchedule);
            graph.adjacencyList.get(i).add(emptyArc);

            if(d == 0) continue;

            Set<Entry<BigInteger, ParetoFront>> schedules = DroneScheduleEnumeration.paretoMap.get(i).get(d).entrySet();
            double baseCapacity = d * Constant.DRONE_AND_EQUIPMENT_WEIGHT;

            for(var entry : schedules) {
                for(DroneSchedule schedule : entry.getValue().nonDominatedSchedules) {
                    double duration = Math.max(servingTime, schedule.makespan);
                    double capacity = baseCapacity;
                    BigInteger customerServed = BigInteger.ZERO.setBit(i);

                    double reducedCost = duration;
                    for(int cust : schedule.customerServed) {
                        reducedCost -= pi[cust-1]; // pi 0-based
                        capacity += VRPInstance.nodes.get(cust).demand;
                        customerServed = customerServed.setBit(cust);
                    }

                    // reducedCost += cuttingPlanes.getReducedCostPenaltyForDroneArc(i, schedule, d);

                    schedule.reducedCost = reducedCost;

                    RCSPArc arc = new RCSPArc(i, to, duration, capacity, reducedCost, customerServed, schedule);
                    graph.adjacencyList.get(i).add(arc);
                }
            }

        }

        return graph;
    }

    private Route reconstructRoute(Label sinkLabel) { // always 0
        List<Integer> sequence = new ArrayList<>();
        Map<Integer, DroneSchedule> droneScheduleMap = new HashMap<>();
        
        Label current = sinkLabel;
        while(current.predecessor != null) {
            RCSPArc arc = current.arc;
            if(arc != null) {

                if(arc.schedule != null)
                    droneScheduleMap.put(arc.src, arc.schedule);

                else if(arc.dst != 0)
                    sequence.add(0, arc.dst);
                    
            }
            current = current.predecessor;
        }

        List<Node> routeSequence = new ArrayList<>();
        for(int cust : sequence) routeSequence.add(VRPInstance.nodes.get(cust));

        Route route = new Route(routeSequence, droneScheduleMap);
        route.reducedCost = sinkLabel.reducedCost;

        return route;
    }    

}
