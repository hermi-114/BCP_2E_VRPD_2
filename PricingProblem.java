import java.math.BigInteger;
import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Deque;
import java.util.HashMap;
import java.util.Iterator;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;
import java.util.Set;

public class PricingProblem {

    // =========================================================================
    // PRICING STAGES
    // =========================================================================
    public enum PricingStage {
        LIGHT,      // stage 1 : one label per bucket (smallest reduced cost)
        HEURISTIC,  // stage 2 : full bucket dominance, ignoring ng-set and cuts
        EXACT       // stage 3 : full dominance including ng-set and cut state
    }

    // =========================================================================
    // FIELDS
    // =========================================================================

    CuttingPlanes cuttingPlanes;

    // One RCSP graph per drone count d = 0 .. MAX_DRONE_PER_VEHICLE
    List<RCSPGraph> graphs;

    // 4-D bucket array: [d][node][Bd][Bc]
    List<Label>[][][][] buckets;
    Label[][][][] dominatingLabels;

    // Debug switch.  Set to true for detailed diagnostics.
    boolean check = false;

    public List<Route> newRoutes = new ArrayList<>();
    Set<String> signatures;

    // ng-neighborhood N_v for each vertex v (bitset of customers).
    BigInteger[] ngNeighborhood;

    private List<R1Cut> activeR1Cuts = new ArrayList<>();


    // Dimensions.
    static final int NUM_NODES  = 2 * (Constant.TOTAL_CUSTOMER + 1);
    static final int NUM_D      = Constant.MAX_DRONE_PER_VEHICLE + 1;
    static final int NUM_BUCKET = Config.SIZE_BUCKET;

    // Global bucket step for the time resource: (horizon) / NUM_BUCKET.
    // Per Sadykov et al. (2021), Section 3.3, the step is a single number
    // covering the whole planning horizon, not per-vertex window size.
    double timeOrigin;   // earliest possible start of service
    double timeStep;     // bucket step for the time resource

    // Global bucket step for the capacity resource.
    double capaStep;

    // =========================================================================
    // CONSTRUCTOR
    // =========================================================================
    public PricingProblem(CuttingPlanes cuttingPlanes, Set<String> usedRoutes) {
        this.cuttingPlanes = cuttingPlanes;
        this.signatures = usedRoutes;
        initNgNeighborhoods();
        initBucketSteps();
        allocateBuckets();
    }

    private static class RCSPGraph {
        List<List<RCSPArc>> adjacencyList;
        RCSPGraph() { adjacencyList = new ArrayList<>(); }
    }

    // =========================================================================
    // INITIALISATION
    // =========================================================================

    /**
     * Initialises N_v for every customer v.  Uses VRPInstance.ngNeighborhood
     * if provided; otherwise falls back to the elementary relaxation
     * (N_v = { all customers }).
     */
    private void initNgNeighborhoods() {
        if (VRPInstance.ngNeighborhood != null) {
            ngNeighborhood = VRPInstance.ngNeighborhood;
            return;
        }
        int n = Constant.TOTAL_CUSTOMER;
        ngNeighborhood = new BigInteger[n + 1];
        BigInteger all = BigInteger.ZERO;
        for (int u = 0; u <= n; u++) all = all.setBit(u);
        for (int v = 0; v <= n; v++) {
            ngNeighborhood[v] = (v == 0) ? BigInteger.ZERO : all;
        }
    }

    /**
     * Global bucket step for the time resource.
     *
     * We define the horizon as the maximum tw_b over all nodes minus the
     * minimum tw_a over all nodes.  The step is horizon / NUM_BUCKET so a
     * route's duration always fits into [0, NUM_BUCKET).
     *
     * Capacity step is TRUCK_PAYLOAD / NUM_BUCKET.
     */
    private void initBucketSteps() {
        double minStart = Double.POSITIVE_INFINITY;
        double maxEnd   = Double.NEGATIVE_INFINITY;
        for (Node node : VRPInstance.nodes) {
            if (node.tw_a < minStart) minStart = node.tw_a;
            if (node.tw_b > maxEnd)   maxEnd   = node.tw_b;
        }
        timeOrigin = (minStart == Double.POSITIVE_INFINITY) ? 0.0 : minStart;
        double horizon = Math.max(maxEnd - timeOrigin, 1.0);
        timeStep = horizon / NUM_BUCKET;

        capaStep = (double) Constant.TRUCK_PAYLOAD / NUM_BUCKET;
    }

    @SuppressWarnings("unchecked")
    private void allocateBuckets() {
        buckets = (List<Label>[][][][]) new List[NUM_D][NUM_NODES][NUM_BUCKET][NUM_BUCKET];
        dominatingLabels = new Label[NUM_D][NUM_NODES][NUM_BUCKET][NUM_BUCKET];
        for (int d = 0; d < NUM_D; d++)
            for (int v = 0; v < NUM_NODES; v++)
                for (int i = 0; i < NUM_BUCKET; i++)
                    for (int j = 0; j < NUM_BUCKET; j++)
                        buckets[d][v][i][j] = new ArrayList<>();
    }

    // =========================================================================
    // THREE-STAGE PRICING ENTRY POINT
    // =========================================================================

    /**
     * Runs the three-stage pricing from Sadykov et al. (2021), Section 5.
     *
     *   Stage 1 (LIGHT)     : one label per bucket.
     *   Stage 2 (HEURISTIC) : full dominance, ignoring ng-set / cut state.
     *   Stage 3 (EXACT)     : full dominance.
     */
    public void runThreeStagePricing(double[] pi, double dualTruck, double dualDrone) {
        newRoutes.clear();
        activeR1Cuts.clear();

        for (ICut cut : cuttingPlanes.cuts)
            if (cut instanceof R1Cut r1Cut) activeR1Cuts.add(r1Cut);

        buildGraphs(pi);

        solveRCSP(dualTruck, dualDrone, PricingStage.LIGHT);
        solveRCSP(dualTruck, dualDrone, PricingStage.HEURISTIC);
        solveRCSP(dualTruck, dualDrone, PricingStage.EXACT);
    }

    // =========================================================================
    // MAIN LABEL-CORRECTING LOOP
    // =========================================================================

    private void solveRCSP(double dualTruck, double dualDrone, PricingStage stage) {

        // Clear buckets and dominating labels for this stage.
        for (int d = 0; d < NUM_D; d++)
            for (int v = 0; v < NUM_NODES; v++)
                for (int i = 0; i < NUM_BUCKET; i++)
                    for (int j = 0; j < NUM_BUCKET; j++) {
                        buckets[d][v][i][j].clear();
                        dominatingLabels[d][v][i][j] = null;
                    }

        int sizeV = Constant.TOTAL_CUSTOMER + 1;

        for (int d = 0; d < NUM_D; d++) {
            RCSPGraph graph = graphs.get(d);

            Deque<Label> queue = new ArrayDeque<>();

            // Source at 0' (prime depot).
            Label srcLabel = new Label(sizeV);
            srcLabel.ngSet     = BigInteger.ZERO;
            srcLabel.d         = d;
            srcLabel.droneUsed = 0;
            srcLabel.r1cState  = new double[activeR1Cuts.size()];
            if (addLabelToBucket(srcLabel, d, stage)) queue.add(srcLabel);

            while (!queue.isEmpty()) {
                Label label = queue.poll();

                // --- sink: complete a route ---
                if (label.node == 0) {
                    if (label.reducedCost < 0) {
                        Route route = reconstructRoute(label);
                        // Use the label's actual drone count, not the loop index.
                        route.reducedCost = label.reducedCost
                                          - dualTruck
                                          - label.droneUsed * dualDrone;
                        String sig = route.getSignature();

                        if (route.reducedCost >= Constant.ROUTE_REDUCED_COST_THRESHOLD || signatures.contains(sig)) {
                            continue;
                        }
                        newRoutes.add(route);
                        signatures.add(sig);
                    }
                    continue;
                }

                // --- extend along all outgoing arcs ---
                for (RCSPArc arc : graph.adjacencyList.get(label.node)) {
                    Label newLabel = extendLabel(label, arc, d);
                    if (newLabel == null) continue;
                    if (addLabelToBucket(newLabel, d, stage)) queue.add(newLabel);
                }
            }

            if (check) {
                int empty = 0;
                for (int v = 0; v < NUM_NODES; v++)
                    for (int i = 0; i < NUM_BUCKET; i++)
                        for (int j = 0; j < NUM_BUCKET; j++)
                            empty += buckets[d][v][i][j].size();
                System.out.println("d=" + d + " stage=" + stage
                    + " total labels still in buckets = " + empty);
            }
        }

        
    }

    // =========================================================================
    // LABEL EXTENSION
    // =========================================================================

    private Label extendLabel(Label label, RCSPArc arc, int d) {
        int dst      = arc.dst;
        int origDst  = dst % (Constant.TOTAL_CUSTOMER + 1);
        Node dstNode = VRPInstance.nodes.get(origDst);

        // --- ng-set feasibility + update ---
        BigInteger newNgSet = label.ngSet;
        for (int cust : arc.servedCustomersInOrder) {
            if (newNgSet.testBit(cust)) {
                if(check) System.out.println("REJECT ng: dst=" + dst + " cust=" + cust);
                return null;
            }
            newNgSet = newNgSet.and(ngNeighborhood[cust])
                               .or(BigInteger.ONE.shiftLeft(cust));
        }

        BigInteger newCustomerServed = label.customerServed;
        for (int cust : arc.servedCustomersInOrder) {
            if (newCustomerServed.testBit(cust)) {
                return null;                     // ← elementary check: reject revisit
            }
            newCustomerServed = newCustomerServed.setBit(cust);
        }

        // --- time-window clamping ---
        int sizeV = Constant.TOTAL_CUSTOMER + 1;
        double arrival = label.duration + arc.duration;
        double newDuration;
        if (dst < sizeV) {
            // Original node: constrain the START of service.
            double startService = Math.max(arrival, dstNode.tw_a);
            if (startService > dstNode.tw_b + Constant.EPSILON) {
                if (check) System.out.println("REJECT time: dst=" + dst + " arrival=" + startService + " tw_b=" + dstNode.tw_b);
                return null;
            }
            newDuration = startService;
        } else {
            // Prime node (i'): the customer's window was already enforced
            // by the truck arc into i.  Departure time is unbounded.
            newDuration = arrival;
        }

        // --- capacity upper bound ---
        double newCapacity = label.capacity + arc.capacity;
        if (newCapacity > Constant.TRUCK_PAYLOAD + Constant.EPSILON) {
            if (check) System.out.println("REJECT cap: dst=" + dst + " cap=" + newCapacity + " limit=" + Constant.TRUCK_PAYLOAD);
            return null;
        }

        // --- drone count update ---
        int newDroneUsed = label.droneUsed;
        if (arc.schedule != null && !arc.servedCustomersInOrder.isEmpty()) {
            // Real drone launch: this arc launched `d` drones.
            newDroneUsed += d;
        }

        // --- reduced cost ---
        double[] newState = label.r1cState.clone();

        double extraPenalty = 0.0;

        // Truck arc: one node visited (dst).
        if (arc.schedule == null) {
            for (int c = 0; c < activeR1Cuts.size(); c++) {
                extraPenalty += activeR1Cuts.get(c).visitNode(arc.dst, newState, c);
            }
        }
        // Drone arc: all schedule customers in order.
        else {
            for (int c = 0; c < activeR1Cuts.size(); c++) {
                R1Cut cut = activeR1Cuts.get(c);
                for (int cust : arc.servedCustomersInOrder) {
                    extraPenalty += cut.visitNode(cust, newState, c);
                }
            }
        }

        // ARCC penalties remain precomputed inside arc.reducedCost.
        double newReducedCost = label.reducedCost + arc.reducedCost + extraPenalty;

        Label newLabel = new Label(arc.dst);
        newLabel.arc          = arc;
        newLabel.duration     = newDuration;
        newLabel.capacity     = newCapacity;
        newLabel.reducedCost  = newReducedCost;
        newLabel.predecessor  = label;
        newLabel.ngSet        = newNgSet;
        newLabel.customerServed = newCustomerServed;
        newLabel.d           = d;
        newLabel.droneUsed    = newDroneUsed;
        newLabel.r1cState     = newState;
        return newLabel;
    }

    // =========================================================================
    // BUCKET INSERTION
    // =========================================================================

    private boolean addLabelToBucket(Label newLabel, int d, PricingStage stage) {
        int v = newLabel.node;

        // Global bucket indices (source / prime / sink all use the same step).
        int Bc = (int) Math.floor( newLabel.capacity / capaStep );
        int Bd = (int) Math.floor((newLabel.duration - timeOrigin) / timeStep);

        if(check) {
            if (newLabel.node == 0 || newLabel.node == Constant.TOTAL_CUSTOMER + 1) {
                System.out.println("bucket check: node=" + newLabel.node
                    + " dur=" + newLabel.duration
                    + " cap=" + newLabel.capacity
                    + " tw_a=" + VRPInstance.nodes.get(v % (Constant.TOTAL_CUSTOMER + 1)).tw_a + " tw_b=" + VRPInstance.nodes.get(v % (Constant.TOTAL_CUSTOMER + 1)).tw_b
                    + " Bd=" + Bd + " Bc=" + Bc
                    + " NUM_BUCKET=" + NUM_BUCKET);
            }
        }

        if (Bd < 0) Bd = 0;
        if (Bc < 0) Bc = 0;
        if (Bd >= NUM_BUCKET || Bc >= NUM_BUCKET) {
            if(check) System.out.println("REJECT bucket: node=" + v + " Bd=" + Bd + " Bc=" + Bc + " NUM_BUCKET=" + NUM_BUCKET);
            return false;
        }

        List<Label> bucket = buckets[d][v][Bd][Bc];

        switch (stage) {
            case LIGHT -> {
                return addLabelLight(newLabel, bucket);
            }
            case HEURISTIC -> {
                return addLabelWithDominance(newLabel, d, v, Bd, Bc, bucket, false);
            }
            case EXACT -> {
                return addLabelWithDominance(newLabel, d, v, Bd, Bc, bucket, true);
            }
        }
        return false;
    }

    private boolean addLabelLight(Label newLabel, List<Label> bucket) {
        if (bucket.isEmpty()) {
            bucket.add(newLabel);
            return true;
        }
        Label best = bucket.get(0);
        if (newLabel.reducedCost < best.reducedCost - Constant.EPSILON) {
            bucket.clear();
            bucket.add(newLabel);
            return true;
        }
        return false;
    }

    /**
     * HEURISTIC / EXACT dominance.
     *
     * Uses dominatingLabels as a bound to prune the search over componentwise-
     * smaller buckets: if cbar_best[b] > newLabel.reducedCost, no label in b
     * (or in its predecessors) can dominate the new label.
     */
    private boolean addLabelWithDominance(Label newLabel, int d, int v,
                                          int Bd, int Bc,
                                          List<Label> bucket,
                                          boolean checkNgAndCuts) {

        // --- 1. same bucket ---
        Iterator<Label> it = bucket.iterator();
        while (it.hasNext()) {
            Label label = it.next();
            if (isDominates(label, newLabel, checkNgAndCuts)) return false;
            if (isDominates(newLabel, label, checkNgAndCuts)) it.remove();
        }

        // --- 2. componentwise smaller-or-equal buckets ---
        for (int i = 0; i <= Bd; i++) {
            for (int j = 0; j <= Bc; j++) {
                if (i == Bd && j == Bc) continue;
                Label best = dominatingLabels[d][v][i][j];
                if (best != null
                        && best.reducedCost > newLabel.reducedCost + Constant.EPSILON) {
                    // Even the best label in this bucket is more expensive than
                    // the new one → nothing in it can dominate.
                    continue;
                }
                for (Label label : buckets[d][v][i][j]) {
                    if (isDominates(label, newLabel, checkNgAndCuts)) return false;
                }
            }
        }

        // --- 3. add ---
        bucket.add(newLabel);

        // --- 4. update best label for this bucket ---
        if (dominatingLabels[d][v][Bd][Bc] == null
                || isDominates(newLabel, dominatingLabels[d][v][Bd][Bc], checkNgAndCuts)) {
            dominatingLabels[d][v][Bd][Bc] = newLabel;
        }
        return true;
    }

    // =========================================================================
    // DOMINANCE
    // =========================================================================

    private boolean isDominates(Label a, Label b, boolean checkNgAndCuts) {
        if (a == null || b == null) return false;
        if (a.node != b.node) return false;
        if (a.duration > b.duration + Constant.EPSILON) return false;
        if (a.capacity > b.capacity + Constant.EPSILON) return false;
        if (a.droneUsed > b.droneUsed) return false;

        if (checkNgAndCuts) {
            if (!b.customerServed.and(a.customerServed).equals(a.customerServed)) return false;
            if (!b.ngSet.and(a.ngSet).equals(a.ngSet)) return false;


            // lm-R1C state dominance: A dominates B only if A is at least as
            // "advanced" on every cut's running sum as B.
            for (int c = 0; c < a.r1cState.length; c++) {
                if (a.r1cState[c] < b.r1cState[c] - Constant.EPSILON) return false;
            }
        }
        return a.reducedCost <= b.reducedCost + Constant.EPSILON;
    }

    // =========================================================================
    // GRAPH CONSTRUCTION
    // =========================================================================

    private void buildGraphs(double[] pi) {
        graphs = new ArrayList<>();
        for (int d = 0; d < NUM_D; d++) {
            graphs.add(buildGraph(d, pi));
        }
    }

    /**
     * Builds the RCSP graph for a fixed drone count d.
     *
     * Node convention:
     *   0  .. n     : original customers (0 = depot)
     *   n+1 .. 2n+1 : prime nodes i'
     *
     * Outgoing arcs:
     *   adjList[i'] : truck arcs i' → j (j ∈ 0..n, j ≠ i)
     *   adjList[i]  : drone arcs i → i'
     */
    private RCSPGraph buildGraph(int d, double[] pi) {
        RCSPGraph graph = new RCSPGraph();
        int sizeV = Constant.TOTAL_CUSTOMER + 1;

        for (int i = 0; i < NUM_NODES; i++) {
            graph.adjacencyList.add(new ArrayList<>());
        }

        // --- truck arcs: i' → j ---
        for (int i = 0; i < sizeV; i++) {
            int from = sizeV + i;
            for (int j = 0; j < sizeV; j++) {
                if (i == j) continue;

                double drivingTime = VRPInstance.distMatrix[i][j] / Constant.TRUCK_SPEED;
                double reducedCost = drivingTime;
                double capacity    = 0.0;
                BigInteger ngSet   = BigInteger.ZERO;
                List<Integer> servedOrder = new ArrayList<>();

                if (j != 0) {
                    reducedCost -= pi[j - 1];
                    reducedCost += cuttingPlanes.getReducedCostPenaltyForTruckArc(i, j, d);
                    capacity     = VRPInstance.nodes.get(j).demand;
                    ngSet        = BigInteger.ONE.shiftLeft(j);
                    servedOrder.add(j);
                }

                RCSPArc arc = new RCSPArc(from, j,
                                          drivingTime, capacity, reducedCost,
                                          ngSet, servedOrder, null);
                graph.adjacencyList.get(from).add(arc);
            }
        }

        // --- drone arcs: i → i' ---
        for (int i = 1; i <= Constant.TOTAL_CUSTOMER; i++) {
            int to = i + sizeV;
            double servingTime = VRPInstance.nodes.get(i).servingTime;

            // Empty drone arc.
            RCSPArc emptyArc = new RCSPArc(i, to,
                                           servingTime, 0.0, servingTime,
                                           BigInteger.ZERO,
                                           new ArrayList<>(),
                                           new DroneSchedule(i));
            graph.adjacencyList.get(i).add(emptyArc);

            if (d == 0) continue;

            Set<Entry<BigInteger, ParetoFront>> schedules = DroneScheduleEnumeration.paretoMap.get(i).get(d).entrySet();
            double baseCapacity = d * Constant.DRONE_AND_EQUIPMENT_WEIGHT;

            for (Entry<BigInteger, ParetoFront> entry : schedules) {
                for (DroneSchedule schedule : entry.getValue().nonDominatedSchedules) {
                    double duration = Math.max(servingTime, schedule.makespan);
                    double capacity = baseCapacity;
                    BigInteger ngSet = BigInteger.ZERO;
                    double reducedCost = duration;

                    for (int cust : schedule.customerServed) {
                        reducedCost -= pi[cust - 1];
                        capacity    += VRPInstance.nodes.get(cust).demand;
                        ngSet        = ngSet.setBit(cust);
                    }
                    reducedCost += cuttingPlanes.getReducedCostPenaltyForDroneArc(i, schedule, d);
                    schedule.reducedCost = reducedCost;

                    List<Integer> servedOrder = new ArrayList<>(schedule.customerServed);

                    RCSPArc arc = new RCSPArc(i, to,
                                              duration, capacity, reducedCost,
                                              ngSet, servedOrder, schedule);
                    graph.adjacencyList.get(i).add(arc);
                }
            }
        }

        if (check) {
            System.out.println("buildGraph d=" + d + " arc counts:");
            for (int v = 0; v < NUM_NODES; v++) {
                int n = graph.adjacencyList.get(v).size();
                if (n > 0) System.out.println("  node " + v + " -> " + n);
            }
        }

        return graph;
    }

    // =========================================================================
    // ROUTE RECONSTRUCTION
    // =========================================================================

    private Route reconstructRoute(Label sinkLabel) {
        List<Integer> sequence = new ArrayList<>();
        Map<Integer, DroneSchedule> droneScheduleMap = new HashMap<>();

        Label current = sinkLabel;
        while (current.predecessor != null) {
            RCSPArc arc = current.arc;
            if (arc != null) {
                if (arc.schedule != null && !arc.servedCustomersInOrder.isEmpty()) {
                    droneScheduleMap.put(arc.src, arc.schedule);
                } else if (arc.schedule == null && arc.dst != 0) {
                    sequence.add(0, arc.dst);
                }
            }
            current = current.predecessor;
        }

        List<Node> routeSequence = new ArrayList<>();
        for (int cust : sequence) routeSequence.add(VRPInstance.nodes.get(cust));

        Route route = new Route(routeSequence, droneScheduleMap);
        route.reducedCost = sinkLabel.reducedCost;
        return route;
    }

    // =========================================================================
    // ACCESSOR
    // =========================================================================

    public List<Route> getNewRoutes() {
        return newRoutes;
    }
}