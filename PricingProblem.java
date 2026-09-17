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

    // ---- CG state ----
    private BigInteger blockedCustomers = BigInteger.ZERO;
    private Set<String> forbiddenSigs   = new HashSet<>();

    // ---- customer-based branching state ----
    private BigInteger forcedTruck = BigInteger.ZERO;
    private BigInteger forcedDrone = BigInteger.ZERO;
    private Map<Integer,Integer> droneFromNode = new HashMap<>();

    // ---- bucket-arc elimination ----
    private final Set<Long> bannedArcs = new HashSet<>();
    private int cgIteration = 0;

    // ---- route enumeration (Baldacci 2008) ----
    // enumeratedByD[d] = all routes of exactly-d-drone type with reduced cost < 0
    private List<List<Route>> enumeratedByD = null;
    private int               enumeratedTotalCount = 0;
    private boolean           enumerationComplete  = false;

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

    // ------------------------------------------------------------------
    //  External setter for the current CG iteration (for arc elimination)
    // ------------------------------------------------------------------
    public void setCgIteration(int it) { this.cgIteration = it; }

    // ------------------------------------------------------------------
    //  Entry point
    // ------------------------------------------------------------------
    public void runThreeStagePricing(double[] pi,
                                     double dualTruck,
                                     double dualDrone,
                                     BigInteger blockedCustomers,
                                     Set<String> forbiddenSigs,
                                     BigInteger forcedTruck,
                                     BigInteger forcedDrone,
                                     Map<Integer,Integer> droneFromNode) {

        this.blockedCustomers = blockedCustomers;
        this.forbiddenSigs    = forbiddenSigs;
        this.forcedTruck      = (forcedTruck == null) ? BigInteger.ZERO : forcedTruck;
        this.forcedDrone      = (forcedDrone == null) ? BigInteger.ZERO : forcedDrone;
        this.droneFromNode    = (droneFromNode == null) ? Collections.emptyMap() : droneFromNode;

        newRoutes.clear();
        activeR1Cuts.clear();
        for (ICut cut : cuttingPlanes.cuts)
            if (cut instanceof R1Cut r) activeR1Cuts.add(r);

        // -------------- Route enumeration shortcut --------------
        if (enumerationComplete) {
            inspectEnumeratedRoutes(pi, dualTruck, dualDrone);
            return;
        }

        // -------------- Normal three-stage pricing --------------
        buildGraphs(pi);

        solveRCSP(dualTruck, dualDrone, PricingStage.LIGHT,
                  Constant.MAX_COLUMNS_LIGHT);

        if (newRoutes.size() < Constant.MAX_COLUMNS_LIGHT) {
            solveRCSP(dualTruck, dualDrone, PricingStage.HEURISTIC,
                      Constant.MAX_COLUMNS_HEURISTIC);

            if (newRoutes.size() < Constant.MAX_COLUMNS_LIGHT
                                       + Constant.MAX_COLUMNS_HEURISTIC) {
                solveRCSP(dualTruck, dualDrone, PricingStage.EXACT,
                          Constant.MAX_COLUMNS_EXACT);
            }
        }

        // -------------- Try route enumeration --------------
        tryRouteEnumeration(pi, dualTruck, dualDrone);
    }

    public void runThreeStagePricing(double[] pi, double dualTruck, double dualDrone) {
        runThreeStagePricing(pi, dualTruck, dualDrone,
                             BigInteger.ZERO, new HashSet<>(),
                             BigInteger.ZERO, BigInteger.ZERO, Collections.emptyMap());
    }

    // ------------------------------------------------------------------
    //  RCSP with per-stage column budget
    // ------------------------------------------------------------------
    private void solveRCSP(double dualTruck, double dualDrone,
                           PricingStage stage, int columnBudget) {

        int alreadyFound = newRoutes.size();
        int targetTotal  = alreadyFound + columnBudget;

        for (int d = 0; d < NUM_D; d++)
            for (int v = 0; v < NUM_NODES; v++)
                for (int i = 0; i < NUM_BUCKET; i++)
                    for (int j = 0; j < NUM_BUCKET; j++) {
                        buckets[d][v][i][j].clear();
                        dominatingLabels[d][v][i][j] = null;
                    }

        int sizeV = Constant.TOTAL_CUSTOMER + 1;

        outer:
        for (int d = 0; d < NUM_D; d++) {
            RCSPGraph graph = graphs.get(d);
            Deque<Label> queue = new ArrayDeque<>();

            Label srcLabel = new Label(sizeV);
            srcLabel.ngSet     = BigInteger.ZERO;
            srcLabel.d         = d;
            srcLabel.droneUsed = 0;
            srcLabel.r1cState  = new double[activeR1Cuts.size()];
            if (addLabelToBucket(srcLabel, d, stage)) queue.add(srcLabel);

            while (!queue.isEmpty()) {
                Label label = queue.poll();

                if (label.node == 0) {
                    if (satisfiesBranchAtSink(label) && label.reducedCost < 0) {
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
                        if (newRoutes.size() >= targetTotal) break outer;
                    }
                    continue;
                }

                for (RCSPArc arc : graph.adjacencyList.get(label.node)) {
                    if (isArcBanned(arc)) continue;
                    Label newLabel = extendLabel(label, arc, d);
                    if (newLabel == null) continue;
                    if (addLabelToBucket(newLabel, d, stage)) queue.add(newLabel);
                }
            }
        }
    }

    private boolean isArcBanned(RCSPArc arc) {
        if (bannedArcs.isEmpty()) return false;
        return bannedArcs.contains(arcKey(arc.src, arc.dst));
    }

    private static long arcKey(int src, int dst) {
        return ((long) src << 32) | (dst & 0xFFFFFFFFL);
    }

    // ------------------------------------------------------------------
    //  Extension + branch enforcement (unchanged from earlier rewrite)
    // ------------------------------------------------------------------
    private Label extendLabel(Label label, RCSPArc arc, int d) {
        int dst     = arc.dst;
        int origDst = dst % (Constant.TOTAL_CUSTOMER + 1);
        Node dstNode = VRPInstance.nodes.get(origDst);

        for (int cust : arc.servedCustomersInOrder)
            if (blockedCustomers.testBit(cust)) return null;

        boolean isRealDroneArc = (arc.schedule != null && !arc.servedCustomersInOrder.isEmpty());
        boolean isTruckVisit   = (arc.schedule == null && !arc.servedCustomersInOrder.isEmpty());

        BigInteger newTruckBit = label.truckBit;
        BigInteger newDroneBit = label.droneBit;

        if (isTruckVisit) {
            for (int c : arc.servedCustomersInOrder) {
                if (forcedDrone.testBit(c)) return null;
                if (droneFromNode.containsKey(c)) return null;
                newTruckBit = newTruckBit.setBit(c);
            }
        } else if (isRealDroneArc) {
            int launchNode = arc.src;
            for (int c : arc.servedCustomersInOrder) {
                if (forcedTruck.testBit(c)) return null;
                Integer requiredU = droneFromNode.get(c);
                if (requiredU != null && requiredU != launchNode) return null;
                newDroneBit = newDroneBit.setBit(c);
            }
        }

        BigInteger newNgSet = label.ngSet;
        for (int cust : arc.servedCustomersInOrder) {
            if (newNgSet.testBit(cust)) return null;
            newNgSet = newNgSet.and(ngNeighborhood[cust])
                               .or(BigInteger.ONE.shiftLeft(cust));
        }

        BigInteger newCustomerServed = label.customerServed;
        for (int cust : arc.servedCustomersInOrder) {
            if (newCustomerServed.testBit(cust)) return null;
            newCustomerServed = newCustomerServed.setBit(cust);
        }

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

        double newCapacity = label.capacity + arc.capacity;
        if (newCapacity > Constant.TRUCK_PAYLOAD + Constant.EPSILON) return null;

        int newDroneUsed = label.droneUsed;
        if (isRealDroneArc) newDroneUsed = Math.max(newDroneUsed, d);

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
        nl.truckBit       = newTruckBit;
        nl.droneBit       = newDroneBit;
        nl.d             = d;
        nl.droneUsed     = newDroneUsed;
        nl.r1cState      = newState;
        return nl;
    }

    private boolean satisfiesBranchAtSink(Label label) {
        if (!label.truckBit.and(forcedTruck).equals(forcedTruck)) return false;
        if (!label.droneBit.and(forcedDrone).equals(forcedDrone)) return false;

        for (Map.Entry<Integer,Integer> e : droneFromNode.entrySet()) {
            int c = e.getKey();
            int u = e.getValue();
            if (label.truckBit.testBit(c)) return false;
            if (!label.droneBit.testBit(c)) return false;
            if (u != 0 && !label.truckBit.testBit(u)) return false;
        }
        return true;
    }

    // ------------------------------------------------------------------
    //  Bucket management (unchanged; dominance includes branch state)
    // ------------------------------------------------------------------
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

    private boolean addLabelWithDominance(Label newLabel, int d, int v, int Bd, int Bc,
                                          List<Label> bucket, boolean checkNgAndCuts) {
        boolean removedBest = false;
        Label curBest = dominatingLabels[d][v][Bd][Bc];
        Iterator<Label> it = bucket.iterator();
        while (it.hasNext()) {
            Label label = it.next();
            if (isDominates(label, newLabel, checkNgAndCuts)) return false;
            if (isDominates(newLabel, label, checkNgAndCuts)) {
                if (label == curBest) removedBest = true;
                it.remove();
            }
        }
        if (removedBest) dominatingLabels[d][v][Bd][Bc] = null;

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
        Label best = dominatingLabels[d][v][Bd][Bc];
        if (best == null || isDominates(newLabel, best, checkNgAndCuts))
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
            if (!b.truckBit.and(a.truckBit).equals(a.truckBit)) return false;
            if (!b.droneBit.and(a.droneBit).equals(a.droneBit)) return false;
            for (int c = 0; c < a.r1cState.length; c++)
                if (a.r1cState[c] > b.r1cState[c] + Constant.EPSILON) return false;
        }
        return a.reducedCost <= b.reducedCost + Constant.EPSILON;
    }

    // ------------------------------------------------------------------
    //  Graph construction
    // ------------------------------------------------------------------
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
                if (bannedArcs.contains(arcKey(from, j))) continue;
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

    // ------------------------------------------------------------------
    //  Route enumeration (Baldacci 2008)
    // ------------------------------------------------------------------
    /**
     * Runs the exact RCSP without a column budget to collect ALL elementary
     * routes with reduced cost < 0. If the total across all d stays under
     * ROUTE_ENUM_THRESHOLD, we keep them and mark enumeration complete.
     * Otherwise we abandon enumeration and fall back to on-demand pricing.
     */
    private void tryRouteEnumeration(double[] pi, double dualTruck, double dualDrone) {
        if (enumerationComplete) return;

        List<List<Route>> byD = new ArrayList<>();
        int total = 0;

        int sizeV = Constant.TOTAL_CUSTOMER + 1;

        for (int d = 0; d < NUM_D; d++) {
            List<Route> routesD = new ArrayList<>();
            RCSPGraph graph = graphs.get(d);
            Deque<Label> queue = new ArrayDeque<>();

            // clear buckets
            for (int v = 0; v < NUM_NODES; v++)
                for (int i = 0; i < NUM_BUCKET; i++)
                    for (int j = 0; j < NUM_BUCKET; j++) {
                        buckets[d][v][i][j].clear();
                        dominatingLabels[d][v][i][j] = null;
                    }

            Label srcLabel = new Label(sizeV);
            srcLabel.ngSet     = BigInteger.ZERO;
            srcLabel.d         = d;
            srcLabel.droneUsed = 0;
            srcLabel.r1cState  = new double[activeR1Cuts.size()];
            if (addLabelToBucket(srcLabel, d, PricingStage.EXACT)) queue.add(srcLabel);

            while (!queue.isEmpty()) {
                Label label = queue.poll();
                if (label.node == 0) {
                    if (satisfiesBranchAtSink(label) && label.reducedCost < -Constant.EPSILON) {
                        Route route = reconstructRoute(label);
                        route.reducedCost = label.reducedCost
                                          - dualTruck
                                          - label.droneUsed * dualDrone;
                        if (route.reducedCost < -Constant.EPSILON) {
                            routesD.add(route);
                            total++;
                            if (total > Constant.ROUTE_ENUM_HARD_CAP) {
                                enumerationComplete = false;
                                enumeratedByD = null;
                                return;   // give up
                            }
                        }
                    }
                    continue;
                }
                for (RCSPArc arc : graph.adjacencyList.get(label.node)) {
                    if (isArcBanned(arc)) continue;
                    Label nl = extendLabel(label, arc, d);
                    if (nl == null) continue;
                    if (addLabelToBucket(nl, d, PricingStage.EXACT)) queue.add(nl);
                }
            }

            byD.add(routesD);
        }

        if (total <= Constant.ROUTE_ENUM_THRESHOLD) {
            this.enumeratedByD         = byD;
            this.enumeratedTotalCount  = total;
            this.enumerationComplete   = true;
            System.out.println("    [ENUM] enumerated " + total
                             + " improving routes (<= "
                             + Constant.ROUTE_ENUM_THRESHOLD + ") → pricing by inspection.");
        } else {
            this.enumeratedByD        = null;
            this.enumeratedTotalCount = 0;
            this.enumerationComplete  = false;
            System.out.println("    [ENUM] enumeration aborted at " + total
                             + " routes (> " + Constant.ROUTE_ENUM_THRESHOLD + ").");
        }
    }

    /**
     * When enumeration succeeded, pricing is just a filter over the stored routes.
     * Only routes whose reduced cost is still negative under the *current* duals
     * are returned.
     */
    private void inspectEnumeratedRoutes(double[] pi, double dualTruck, double dualDrone) {
        for (List<Route> routesD : enumeratedByD) {
            for (Route r : routesD) {
                double rc = computeReducedCost(r, pi, dualTruck, dualDrone);
                if (rc < -Constant.EPSILON) {
                    r.reducedCost = rc;
                    newRoutes.add(r);
                }
            }
        }
    }

    private double computeReducedCost(Route r, double[] pi,
                                      double dualTruck, double dualDrone) {
        // c_r = totalTime(r) - Σ pi_i - dualTruck - numDrones * dualDrone
        double rc = r.totalTime;
        // subtract Σ pi over served customers
        List<Integer> served = new ArrayList<>(r.customerServed);
        for (int c : served) rc -= pi[c - 1];
        rc -= dualTruck;
        rc -= r.getNumDrone() * dualDrone;
        return rc;
    }

    public boolean isEnumerationComplete() { return enumerationComplete; }
    public int     getEnumeratedTotalCount() { return enumeratedTotalCount; }
    public List<List<Route>> getEnumeratedByD() { return enumeratedByD; }

    public List<Route> getNewRoutes() { return newRoutes; }
}