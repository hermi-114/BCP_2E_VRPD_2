import java.math.BigInteger;
import java.util.*;

public class PricingProblem {

    public enum PricingStage { LIGHT, HEURISTIC, EXACT }

    CuttingPlanes cuttingPlanes;

    List<RCSPGraph> graphs;
    List<Label>[][][][] buckets;
    Label[][][][] dominatingLabels;


    public List<Route> newRoutes = new ArrayList<>();
    Set<String> signatures;

    BigInteger[] ngNeighborhood;
    private List<R1Cut> activeR1Cuts = new ArrayList<>();

    // NEW: branch-and-price state
    private BigInteger blockedCustomers   = BigInteger.ZERO;
    private Set<String> forbiddenSigs     = new HashSet<>();

    static final int NUM_NODES  = 2 * (Constant.TOTAL_CUSTOMER + 1);
    static final int NUM_D      = Constant.MAX_DRONE_PER_VEHICLE + 1;
    static final int NUM_BUCKET = Config.SIZE_BUCKET;

    double timeOrigin;
    double timeStep;
    double capaStep;

    public PricingProblem(CuttingPlanes cuttingPlanes, Set<String> usedRoutes) {
        this.cuttingPlanes = cuttingPlanes;
        this.signatures    = usedRoutes;
        initNgNeighborhoods();
        initBucketSteps();
        allocateBuckets();
    }

    private static class RCSPGraph {
        List<List<RCSPArc>> adjacencyList = new ArrayList<>();
    }

    private void initNgNeighborhoods() {
        if (VRPInstance.ngNeighborhood != null) {
            ngNeighborhood = VRPInstance.ngNeighborhood;
            return;
        }
        int n = Constant.TOTAL_CUSTOMER;
        ngNeighborhood = new BigInteger[n + 1];
        BigInteger all = BigInteger.ZERO;
        for (int u = 0; u <= n; u++) all = all.setBit(u);
        for (int v = 0; v <= n; v++) ngNeighborhood[v] = (v == 0) ? BigInteger.ZERO : all;
    }

    private void initBucketSteps() {
        double minStart = Double.POSITIVE_INFINITY, maxEnd = Double.NEGATIVE_INFINITY;
        for (Node node : VRPInstance.nodes) {
            if (node.tw_a < minStart) minStart = node.tw_a;
            if (node.tw_b > maxEnd)   maxEnd   = node.tw_b;
        }
        timeOrigin = Double.isInfinite(minStart) ? 0.0 : minStart;
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

    // NEW signature with blocked customers + forbidden signatures
    public void runThreeStagePricing(double[] pi, double dualTruck, double dualDrone,
                                     BigInteger blockedCustomers,
                                     Set<String> forbiddenSigs) {
        this.blockedCustomers = blockedCustomers;
        this.forbiddenSigs    = forbiddenSigs;

        newRoutes.clear();
        activeR1Cuts.clear();
        for (ICut cut : cuttingPlanes.cuts)
            if (cut instanceof R1Cut r) activeR1Cuts.add(r);

        buildGraphs(pi);
        solveRCSP(dualTruck, dualDrone, PricingStage.LIGHT);
        solveRCSP(dualTruck, dualDrone, PricingStage.HEURISTIC);
        solveRCSP(dualTruck, dualDrone, PricingStage.EXACT);
    }

    /** Backwards-compatible overload (no branch constraints). */
    public void runThreeStagePricing(double[] pi, double dualTruck, double dualDrone) {
        runThreeStagePricing(pi, dualTruck, dualDrone,
                             BigInteger.ZERO, new HashSet<>());
    }

    private void solveRCSP(double dualTruck, double dualDrone, PricingStage stage) {
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

            Label srcLabel = new Label(sizeV);
            srcLabel.ngSet      = BigInteger.ZERO;
            srcLabel.d          = d;
            srcLabel.droneUsed  = 0;
            srcLabel.r1cState   = new double[activeR1Cuts.size()];
            if (addLabelToBucket(srcLabel, d, stage)) queue.add(srcLabel);

            while (!queue.isEmpty()) {
                Label label = queue.poll();

                if (label.node == 0) {
                    if (label.reducedCost < 0) {
                        Route route = reconstructRoute(label);
                        route.reducedCost = label.reducedCost
                                          - dualTruck
                                          - label.droneUsed * dualDrone;
                        String sig = route.getSignature();
                        if (route.reducedCost >= Constant.ROUTE_REDUCED_COST_THRESHOLD
                                || signatures.contains(sig)
                                || forbiddenSigs.contains(sig)) continue;
                        newRoutes.add(route);
                        signatures.add(sig);
                    }
                    continue;
                }

                for (RCSPArc arc : graph.adjacencyList.get(label.node)) {
                    Label newLabel = extendLabel(label, arc, d);
                    if (newLabel == null) continue;
                    if (addLabelToBucket(newLabel, d, stage)) queue.add(newLabel);
                }
            }
        }
    }

    private Label extendLabel(Label label, RCSPArc arc, int d) {
        int dst     = arc.dst;
        int origDst = dst % (Constant.TOTAL_CUSTOMER + 1);
        Node dstNode = VRPInstance.nodes.get(origDst);

        // NEW: block customers already covered by a forced route.
        for (int cust : arc.servedCustomersInOrder) {
            if (blockedCustomers.testBit(cust)) return null;
        }

        // ng-set
        BigInteger newNgSet = label.ngSet;
        for (int cust : arc.servedCustomersInOrder) {
            if (newNgSet.testBit(cust)) return null;
            newNgSet = newNgSet.and(ngNeighborhood[cust])
                               .or(BigInteger.ONE.shiftLeft(cust));
        }

        // elementary check
        BigInteger newCustomerServed = label.customerServed;
        for (int cust : arc.servedCustomersInOrder) {
            if (newCustomerServed.testBit(cust)) return null;
            newCustomerServed = newCustomerServed.setBit(cust);
        }

        // time window
        int sizeV = Constant.TOTAL_CUSTOMER + 1;
        double arrival = label.duration + arc.duration;
        double newDuration;
        if (dst < sizeV) {
            double startService = Math.max(arrival, dstNode.tw_a);
            if (startService > dstNode.tw_b + Constant.EPSILON) return null;
            newDuration = startService;
        } else {
            newDuration = arrival;
        }

        // capacity
        double newCapacity = label.capacity + arc.capacity;
        if (newCapacity > Constant.TRUCK_PAYLOAD + Constant.EPSILON) return null;

        // drone count
        int newDroneUsed = label.droneUsed;
        if (arc.schedule != null && !arc.servedCustomersInOrder.isEmpty()) newDroneUsed += d;

        // lm-R1C state
        double[] newState = label.r1cState.clone();
        double extraPenalty = 0.0;
        if (arc.schedule == null) {
            for (int c = 0; c < activeR1Cuts.size(); c++)
                extraPenalty += activeR1Cuts.get(c).visitNode(arc.dst, newState, c);
        } else {
            for (int c = 0; c < activeR1Cuts.size(); c++) {
                R1Cut cut = activeR1Cuts.get(c);
                for (int cust : arc.servedCustomersInOrder)
                    extraPenalty += cut.visitNode(cust, newState, c);
            }
        }
        double newReducedCost = label.reducedCost + arc.reducedCost + extraPenalty;

        Label nl = new Label(arc.dst);
        nl.arc            = arc;
        nl.duration       = newDuration;
        nl.capacity       = newCapacity;
        nl.reducedCost    = newReducedCost;
        nl.predecessor    = label;
        nl.ngSet          = newNgSet;
        nl.customerServed = newCustomerServed;
        nl.d              = d;
        nl.droneUsed      = newDroneUsed;
        nl.r1cState       = newState;
        return nl;
    }

    private boolean addLabelToBucket(Label newLabel, int d, PricingStage stage) {
        int v  = newLabel.node;
        int Bc = (int) Math.floor( newLabel.capacity / capaStep );
        int Bd = (int) Math.floor((newLabel.duration - timeOrigin) / timeStep);
        if (Bd < 0) Bd = 0;
        if (Bc < 0) Bc = 0;
        if (Bd >= NUM_BUCKET || Bc >= NUM_BUCKET) return false;

        List<Label> bucket = buckets[d][v][Bd][Bc];
        switch (stage) {
            case LIGHT     -> { return addLabelLight(newLabel, bucket); }
            case HEURISTIC -> { return addLabelWithDominance(newLabel, d, v, Bd, Bc, bucket, false); }
            case EXACT     -> { return addLabelWithDominance(newLabel, d, v, Bd, Bc, bucket, true); }
        }
        return false;
    }

    private boolean addLabelLight(Label newLabel, List<Label> bucket) {
        if (bucket.isEmpty()) { bucket.add(newLabel); return true; }
        Label best = bucket.get(0);
        if (newLabel.reducedCost < best.reducedCost - Constant.EPSILON) {
            bucket.clear();
            bucket.add(newLabel);
            return true;
        }
        return false;
    }

    private boolean addLabelWithDominance(Label newLabel, int d, int v,
                                          int Bd, int Bc, List<Label> bucket,
                                          boolean checkNgAndCuts) {
        Iterator<Label> it = bucket.iterator();
        while (it.hasNext()) {
            Label label = it.next();
            if (isDominates(label, newLabel, checkNgAndCuts)) return false;
            if (isDominates(newLabel, label, checkNgAndCuts)) it.remove();
        }
        for (int i = 0; i <= Bd; i++) {
            for (int j = 0; j <= Bc; j++) {
                if (i == Bd && j == Bc) continue;
                Label best = dominatingLabels[d][v][i][j];
                if (best != null
                        && best.reducedCost > newLabel.reducedCost + Constant.EPSILON)
                    continue;
                for (Label label : buckets[d][v][i][j])
                    if (isDominates(label, newLabel, checkNgAndCuts)) return false;
            }
        }
        bucket.add(newLabel);
        if (dominatingLabels[d][v][Bd][Bc] == null
                || isDominates(newLabel, dominatingLabels[d][v][Bd][Bc], checkNgAndCuts))
            dominatingLabels[d][v][Bd][Bc] = newLabel;
        return true;
    }

    private boolean isDominates(Label a, Label b, boolean checkNgAndCuts) {
        if (a == null || b == null) return false;
        if (a.node != b.node) return false;
        if (a.duration > b.duration + Constant.EPSILON) return false;
        if (a.capacity > b.capacity + Constant.EPSILON) return false;
        if (a.droneUsed > b.droneUsed) return false;
        if (checkNgAndCuts) {
            if (!b.customerServed.and(a.customerServed).equals(a.customerServed)) return false;
            if (!b.ngSet.and(a.ngSet).equals(a.ngSet)) return false;
            for (int c = 0; c < a.r1cState.length; c++)
                if (a.r1cState[c] < b.r1cState[c] - Constant.EPSILON) return false;
        }
        return a.reducedCost <= b.reducedCost + Constant.EPSILON;
    }

    private void buildGraphs(double[] pi) {
        graphs = new ArrayList<>();
        for (int d = 0; d < NUM_D; d++) graphs.add(buildGraph(d, pi));
    }

    private RCSPGraph buildGraph(int d, double[] pi) {
        RCSPGraph graph = new RCSPGraph();
        int sizeV = Constant.TOTAL_CUSTOMER + 1;
        for (int i = 0; i < NUM_NODES; i++) graph.adjacencyList.add(new ArrayList<>());

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
                graph.adjacencyList.get(from).add(new RCSPArc(from, j,
                        drivingTime, capacity, reducedCost, ngSet, servedOrder, null));
            }
        }

        for (int i = 1; i <= Constant.TOTAL_CUSTOMER; i++) {
            int to = i + sizeV;
            double servingTime = VRPInstance.nodes.get(i).servingTime;
            graph.adjacencyList.get(i).add(new RCSPArc(i, to, servingTime, 0.0,
                    servingTime, BigInteger.ZERO, new ArrayList<>(), new DroneSchedule(i)));

            if (d == 0) continue;
            if (i >= DroneScheduleEnumeration.paretoMap.size()) continue;
            if (d >= DroneScheduleEnumeration.paretoMap.get(i).size()) continue;

            var schedules = DroneScheduleEnumeration.paretoMap.get(i).get(d).entrySet();
            double baseCapacity = d * Constant.DRONE_AND_EQUIPMENT_WEIGHT;
            for (var entry : schedules) {
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
                    graph.adjacencyList.get(i).add(new RCSPArc(i, to, duration,
                            capacity, reducedCost, ngSet,
                            new ArrayList<>(schedule.customerServed), schedule));
                }
            }
        }
        return graph;
    }

    private Route reconstructRoute(Label sinkLabel) {
        List<Integer> sequence = new ArrayList<>();
        Map<Integer, DroneSchedule> droneScheduleMap = new HashMap<>();
        Label current = sinkLabel;
        while (current.predecessor != null) {
            RCSPArc arc = current.arc;
            if (arc != null) {
                if (arc.schedule != null && !arc.servedCustomersInOrder.isEmpty())
                    droneScheduleMap.put(arc.src, arc.schedule);
                else if (arc.schedule == null && arc.dst != 0)
                    sequence.add(0, arc.dst);
            }
            current = current.predecessor;
        }
        List<Node> routeSequence = new ArrayList<>();
        for (int c : sequence) routeSequence.add(VRPInstance.nodes.get(c));
        Route route = new Route(routeSequence, droneScheduleMap);
        route.reducedCost = sinkLabel.reducedCost;
        return route;
    }

    public List<Route> getNewRoutes() { return newRoutes; }

}