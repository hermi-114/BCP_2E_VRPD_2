import com.gurobi.gurobi.GRBException;
import java.util.*;
import java.util.concurrent.*;

public class BranchAndPrice {

    private List<Route> bestSolution  = null;   // incumbent (may be suboptimal)
    private double      minTime       = Double.MAX_VALUE;  // primal
    private double      dualObjective = Double.NaN;        // best global LB
    private final ColumnGeneration cg = new ColumnGeneration();

    private int nodesExplored = 0;
    private int branchesMade  = 0;

    private final Map<BranchCandidate, Double> history = new HashMap<>();

    private static final int MAX_NODES = 200;

    public void run() throws GRBException {
        dualObjective = Double.NaN;

        System.out.println("============ B&P (strong branching, pulse) ============");
        long globalStart = System.currentTimeMillis();

        // ================== ROOT ==================
        BCPNode root = new BCPNode();
        ColumnGeneration.NodeResult rootLp = cg.solveForNode(root);
        nodesExplored++;

        System.out.printf("--- Root LP: obj=%.4f  covered=%b  optimal=%b  time=%dms%n",
                rootLp.objective, rootLp.allCovered, rootLp.lpOptimal,
                System.currentTimeMillis() - globalStart);

        if (!rootLp.lpOptimal) {
            dualObjective = Double.POSITIVE_INFINITY;   // infeasible region
            System.out.println("Root LP infeasible — done.");
            return;
        }
        if (!rootLp.allCovered) {
            dualObjective = Double.NaN;                 // no valid bound for original problem
            System.out.println("Root uses artificials — instance infeasible at this configuration.");
            System.out.print("Root: uncovered customers = [");
            for (int c = 0; c < rootLp.artificial.length; c++)
                if (rootLp.artificial[c] > Constant.EPSILON)
                    System.out.print((c + 1) + " ");
            System.out.println("]");
            return;
        }
        if (isIntegral(rootLp.lambda)) {
            minTime       = rootLp.objective;
            bestSolution  = extractSolution(rootLp);
            dualObjective = minTime;
            System.out.println("Root integral — optimal: " + minTime);
            return;
        }

        // ================== ROOT MIP DIAGNOSTIC ==================
        try {
            ColumnGeneration.NodeResult mip = cg.solveForNodeAsMip(root, rootLp.columns);
            if (mip.lpOptimal && mip.allCovered) {
                minTime      = mip.objective;
                bestSolution = extractSolution(mip);
                System.out.printf("Initial incumbent from root MIP: %.4f%n", minTime);
            } else {
                System.out.println("Root MIP did not find a feasible integer solution.");
            }
        } catch (GRBException e) {
            System.out.println("Root MIP failed: " + e.getMessage());
        }

        // ================== ROOT STRONG BRANCH ==================
        BranchDecision rootBranch = pickBranchByStrongBranching(rootLp, root);
        if (rootBranch == null) {
            System.out.println("No branch candidate at root — done.");
            if (bestSolution != null) {
                dualObjective = minTime;            // proven: no better integer exists
                System.out.println("Incumbent stands: " + minTime);
            } else {
                // No incumbent and no way to branch — cannot give a valid bound
                dualObjective = Double.NaN;
            }
            return;
        }
        System.out.println("Root branch: " + rootBranch);

        Deque<BCPNode> stack = new ArrayDeque<>();

        BCPNode childDown = root.copy();
        childDown.decisions.add(new BranchDecision(rootBranch.candidate, true,  Math.floor(rootBranch.rhs)));
        childDown.depth = 1;
        childDown.parentBound = rootLp.objective;

        BCPNode childUp = root.copy();
        childUp.decisions.add(new BranchDecision(rootBranch.candidate, false, Math.ceil(rootBranch.rhs)));
        childUp.depth = 1;
        childUp.parentBound = rootLp.objective;

        stack.push(childUp);
        stack.push(childDown);
        branchesMade++;

        // ================== TREE SEARCH ==================
        while (!stack.isEmpty() && nodesExplored < MAX_NODES) {
            BCPNode node = stack.pop();
            nodesExplored++;

            long t0 = System.currentTimeMillis();
            ColumnGeneration.NodeResult lp = cg.solveForNode(node);
            long elapsed = System.currentTimeMillis() - t0;

            System.out.println("--- Node #" + nodesExplored
                    + " depth=" + node.depth
                    + " dec=" + node.decisions.size()
                    + "  time=" + elapsed + "ms"
                    + "  obj=" + String.format("%.4f", lp.objective)
                    + "  covered=" + lp.allCovered
                    + "  integral=" + isIntegral(lp.lambda));

            if (!lp.lpOptimal) {
                System.out.println("    prune: infeasible");
                continue;
            }
            if (lp.objective >= minTime - Constant.EPSILON) {
                System.out.println("    prune: bound (" + lp.objective + " >= " + minTime + ")");
                continue;
            }
            if (!lp.allCovered) {
                System.out.println("    prune: artificials");
                continue;
            }
            if (isIntegral(lp.lambda)) {
                if (lp.objective < minTime) {
                    minTime = lp.objective;
                    bestSolution = extractSolution(lp);
                    System.out.println("    new incumbent: " + minTime);
                }
                continue;
            }

            BranchDecision bd = pickBranchByStrongBranching(lp, node);
            if (bd == null) {
                // Cannot branch further. Treat as closed; do NOT silently keep open.
                System.out.println("    no candidate — node closed (bound " + lp.objective + ")");
                continue;
            }
            System.out.println("    branch: " + bd);

            BCPNode cd = node.copy();
            cd.decisions.add(new BranchDecision(bd.candidate, true,  Math.floor(bd.rhs)));
            cd.depth = node.depth + 1;
            cd.parentBound = lp.objective;

            BCPNode cu = node.copy();
            cu.decisions.add(new BranchDecision(bd.candidate, false, Math.ceil(bd.rhs)));
            cu.depth = node.depth + 1;
            cu.parentBound = lp.objective;

            stack.push(cu);
            stack.push(cd);
            branchesMade++;
        }

        // ================== FINAL DUAL BOUND ==================
        // If the tree is exhausted (stack empty and node cap not reached),
        // every region is either infeasible, integral, or pruned by bound,
        // so the incumbent is proven optimal → dual == primal.
        // Otherwise the global LB is the minimum parent bound over the open nodes.
        boolean exhausted = stack.isEmpty() && nodesExplored < MAX_NODES;

        if (exhausted) {
            dualObjective = (bestSolution == null) ? Double.POSITIVE_INFINITY : minTime;
        } else {
            double lb = Double.POSITIVE_INFINITY;
            for (BCPNode n : stack) lb = Math.min(lb, n.parentBound);
            if (bestSolution != null) lb = Math.min(lb, minTime);   // cannot exceed incumbent
            dualObjective = lb;
        }

        // ================== REPORT ==================
        System.out.println("=================== Done ===================");
        if (bestSolution == null) {
            System.out.println("No integer solution found within " + MAX_NODES + " nodes.");
        } else {
            System.out.println("Primal (best found): " + minTime);
        }
        System.out.println("Dual   (lower bound): " + dualObjective);
        if (bestSolution != null && Double.isFinite(dualObjective)) {
            double gap = Math.abs(minTime - dualObjective) / Math.abs(minTime) * 100.0;
            System.out.printf("Gap: %.4f%%%s%n",
                    gap,
                    gap < 1e-4 ? "  (proven optimal)" : "  (optimality not proven)");
        }
        System.out.println("Nodes: " + nodesExplored + "  Branches: " + branchesMade);
        System.out.println("Total time: " + (System.currentTimeMillis() - globalStart) / 1000.0 + "s");
    }

    // ------------------------------------------------------------------
    // Strong branching with wall-clock limit + parallel evaluation
    // ------------------------------------------------------------------
    private BranchDecision pickBranchByStrongBranching(ColumnGeneration.NodeResult lp, BCPNode node) throws GRBException {
        long sbStart = System.currentTimeMillis();

        List<BranchCandidate> all = collectCandidates(lp);
        List<BranchCandidate> phase1 = selectPhase1(all, lp);
        if (phase1.isEmpty()) return null;

        List<ScoredCandidate> phase2 = evaluatePhase2(phase1, lp, node);
        phase2.sort((a, b) -> Double.compare(b.score, a.score));
        int keep2 = Math.min(Constant.STRONG_BRANCHING_PHASE2_KEEP, phase2.size());
        List<ScoredCandidate> phase2Top = new ArrayList<>(phase2.subList(0, keep2));

        if (System.currentTimeMillis() - sbStart > Constant.SB_TIME_LIMIT_MS / 2) {
            System.out.println("    [SB] time budget reached at phase 2");
            ScoredCandidate best = phase2Top.get(0);
            history.merge(best.candidate, best.score,
                          (a, b) -> a * Constant.STRONG_BRANCHING_HISTORY_DECAY + b);
            return new BranchDecision(best.candidate, true, best.value);
        }

        List<ScoredCandidate> phase3 = evaluatePhase3(phase2Top, lp, node);
        phase3.removeIf(s -> s.score < 0);
        if (phase3.isEmpty()) return null;
        phase3.sort((a, b) -> Double.compare(b.score, a.score));
        ScoredCandidate best = phase3.get(0);

        history.merge(best.candidate, best.score,
                      (a, b) -> a * Constant.STRONG_BRANCHING_HISTORY_DECAY + b);

        return new BranchDecision(best.candidate, true, best.value);
    }

    private List<BranchCandidate> collectCandidates(ColumnGeneration.NodeResult lp) {
        List<BranchCandidate> list = new ArrayList<>();

        Set<Long> seen = new HashSet<>();
        for (int r = 0; r < lp.columns.size(); r++) {
            if (lp.lambda[r] < Constant.EPSILON) continue;
            List<Node> seq = lp.columns.get(r).sequence;
            for (int k = 1; k < seq.size(); k++) {
                int i = seq.get(k - 1).id, j = seq.get(k).id;
                if (i == 0 && j == 0) continue;
                long key = ((long) i << 32) | (j & 0xFFFFFFFFL);
                if (seen.add(key)) {
                    list.add(new BranchCandidate(BranchCandidate.Type.TRUCK_ARC, i, j));
                    if (list.size() > Constant.MAX_ARC_CANDIDATES_PER_NODE) return list;
                }
            }
        }
        return list;
    }

    private List<BranchCandidate> selectPhase1(List<BranchCandidate> all, ColumnGeneration.NodeResult lp) {
        List<ScoredCandidate> scored = new ArrayList<>();
        for (BranchCandidate c : all) {
            double v = currentValue(c, lp);
            if (v < Constant.EPSILON) continue;
            double frac = Math.abs(v - Math.round(v));
            if (frac < 1e-4) continue;
            double s = frac;
            Double hist = history.get(c);
            if (hist != null) s = Math.max(s, hist);
            scored.add(new ScoredCandidate(c, s, v));
        }
        scored.sort((a, b) -> Double.compare(b.score, a.score));

        int keep = Constant.STRONG_BRANCHING_PHASE1_KEEP;
        Set<BranchCandidate> chosen = new LinkedHashSet<>();
        int half = keep / 2;
        for (int i = 0; i < Math.min(half, scored.size()); i++)
            chosen.add(scored.get(i).candidate);

        Map<BranchCandidate.Type, Integer> tc = new EnumMap<>(BranchCandidate.Type.class);
        for (ScoredCandidate s : scored) {
            if (chosen.size() >= keep) break;
            if (chosen.contains(s.candidate)) continue;
            int cnt = tc.getOrDefault(s.candidate.type, 0);
            if (cnt >= keep / 4 + 1) continue;
            chosen.add(s.candidate);
            tc.merge(s.candidate.type, 1, Integer::sum);
        }
        for (ScoredCandidate s : scored) {
            if (chosen.size() >= keep) break;
            chosen.add(s.candidate);
        }
        return new ArrayList<>(chosen);
    }

    private List<ScoredCandidate> evaluatePhase2(List<BranchCandidate> cands,
                                                 ColumnGeneration.NodeResult lp,
                                                 BCPNode node) throws GRBException {
        return evaluateParallel(cands, lp, node, true);
    }

    private List<ScoredCandidate> evaluatePhase3(List<ScoredCandidate> cands,
                                                 ColumnGeneration.NodeResult lp,
                                                 BCPNode node) throws GRBException {
        List<BranchCandidate> bc = new ArrayList<>();
        for (ScoredCandidate s : cands) bc.add(s.candidate);
        return evaluateParallel(bc, lp, node, true);
    }

    private List<ScoredCandidate> evaluateParallel(List<BranchCandidate> cands,
                                                   ColumnGeneration.NodeResult lp,
                                                   BCPNode node,
                                                   boolean useParallel) throws GRBException {
        double z = lp.objective;
        List<Route> parentCols = lp.columns;

        if (useParallel && Constant.PARALLEL_SB && cands.size() > 1) {
            int nThreads = Math.min(cands.size(),
                    Runtime.getRuntime().availableProcessors());
            ExecutorService pool = Executors.newFixedThreadPool(nThreads);
            List<Future<ScoredCandidate>> futures = new ArrayList<>();

            for (BranchCandidate c : cands) {
                futures.add(pool.submit(() -> {
                    double v = currentValue(c, lp);
                    double zDown, zUp;
                    try {
                        zDown = evaluateBranchSide(parentCols, node, c, true,  Math.floor(v));
                    } catch (GRBException e) { zDown = Double.POSITIVE_INFINITY; }
                    try {
                        zUp = evaluateBranchSide(parentCols, node, c, false, Math.ceil(v));
                    } catch (GRBException e) { zUp = Double.POSITIVE_INFINITY; }
                    return new ScoredCandidate(c, productRule(z, zDown, zUp), v, zDown, zUp);
                }));
            }

            List<ScoredCandidate> out = new ArrayList<>();
            for (Future<ScoredCandidate> f : futures) {
                try { out.add(f.get()); }
                catch (Exception e) { /* skip */ }
            }
            pool.shutdown();
            return out;
        }

        List<ScoredCandidate> out = new ArrayList<>();
        for (BranchCandidate c : cands) {
            double v = currentValue(c, lp);
            double zDown = evaluateBranchSide(parentCols, node, c, true,  Math.floor(v));
            double zUp   = evaluateBranchSide(parentCols, node, c, false, Math.ceil(v));
            out.add(new ScoredCandidate(c, productRule(z, zDown, zUp), v, zDown, zUp));
        }
        return out;
    }

    private double evaluateBranchSide(List<Route> parentColumns,
                                      BCPNode parent,
                                      BranchCandidate cand,
                                      boolean upperBound,
                                      double rhs) throws GRBException {
        BCPNode temp = parent.copy();
        temp.decisions.add(new BranchDecision(cand, upperBound, rhs));
        ColumnGeneration.NodeResult r = cg.solveForNodeWithoutCG(temp, parentColumns);
        return r.lpOptimal ? r.objective : Double.POSITIVE_INFINITY;
    }

    private double productRule(double z, double zDown, double zUp) {
        boolean dI = Double.isInfinite(zDown), uI = Double.isInfinite(zUp);
        if (dI && uI) return -1.0;
        double iD = dI ? Constant.STRONG_BRANCHING_INFEAS_SCORE
                       : Math.max(zDown - z, Constant.EPSILON);
        double iU = uI ? Constant.STRONG_BRANCHING_INFEAS_SCORE
                       : Math.max(zUp - z, Constant.EPSILON);
        return iD * iU;
    }

    private double currentValue(BranchCandidate c, ColumnGeneration.NodeResult lp) {
        double s = 0.0;
        for (int r = 0; r < lp.columns.size(); r++)
            s += lp.lambda[r] * c.coefficient(lp.columns.get(r));
        return s;
    }

    private boolean isIntegral(double[] lambda) {
        for (double v : lambda)
            if (Math.abs(v - Math.round(v)) > 1e-5) return false;
        return true;
    }

    private List<Route> extractSolution(ColumnGeneration.NodeResult lp) {
        List<Route> sol = new ArrayList<>();
        for (int i = 0; i < lp.lambda.length; i++)
            if (lp.lambda[i] > 1 - Constant.EPSILON)
                sol.add(lp.columns.get(i));
        return sol;
    }

    public List<Route> getBestSolution()  { return bestSolution; }
    public double      getBestObjective() { return minTime; }
    public double      getDualObjective() { return dualObjective; }

    private static class ScoredCandidate {
        final BranchCandidate candidate;
        final double score, value, zDown, zUp;

        ScoredCandidate(BranchCandidate c, double s, double v) {
            this(c, s, v, Double.NaN, Double.NaN);
        }

        ScoredCandidate(BranchCandidate c, double s, double v, double zd, double zu) {
            candidate = c; score = s; value = v; zDown = zd; zUp = zu;
        }
    }
}