import java.util.*;

public class R1Cut implements ICut {
    public final Set<Integer> subsetC;
    public final Set<Integer> memorySet;      // M, with C ⊆ M ⊆ V₊
    public final int rhs;
    public final Map<Integer, Double> rhoMap;
    public double dual = 0.0;

    /** Constructor with M = C (simplest form). */
    public R1Cut(Set<Integer> C) {
        this(C, new HashSet<>(C));
    }

    /** Constructor with explicit memory set. */
    public R1Cut(Set<Integer> C, Set<Integer> M) {
        this.subsetC  = new HashSet<>(C);
        this.memorySet = new HashSet<>(M);
        this.rhoMap   = new HashMap<>();
        for (int cust : C) rhoMap.put(cust, 0.5);

        double sumRho = C.size() * 0.5;
        this.rhs = (int) Math.floor(sumRho + Constant.EPSILON);
    }

    @Override public double getDual()           { return dual; }
    @Override public void setDual(double d)     { this.dual = d; }
    @Override public double getRHS()            { return rhs; }
    @Override public Set<Integer> getSubsetC()  { return subsetC; }

    /** Column coefficient in the cut for a finished route (α for the whole path). */
    @Override
    public double getCoefficientForRoute(Route route) {
        double s = 0.0, alpha = 0.0;
        for (Node n : route.sequence) {
            int id = n.id;
            if (id == -1) continue;
            if (!memorySet.contains(id)) s = 0.0;
            Double rho = rhoMap.get(id);
            if (rho != null) s += rho;
            while (s >= 1.0 - Constant.EPSILON) { s -= 1.0; alpha += 1.0; }
        }
        return alpha;
    }

    /**
     * Applies the state update for visiting `node` given current `s`.
     * Returns the penalty contribution (already multiplied by -dual) and
     * writes the new state back into `sHolder[cutIndex]`.
     */
    public double visitNode(int node, double[] sHolder, int cutIndex) {
        double s = sHolder[cutIndex];
        if (!memorySet.contains(node)) s = 0.0;         // memory reset
        Double rho = rhoMap.get(node);
        if (rho != null) s += rho;
        double penalty = 0.0;
        while (s >= 1.0 - Constant.EPSILON) {
            s -= 1.0;
            penalty -= dual;                             // = -dual * 1
        }
        sHolder[cutIndex] = s;
        return penalty;
    }

    @Override
    public double getReducedCostPenaltyForTruckArc(int src, int dst, int numDrones) {
        // Not used any more (stateful penalty applied at extension)
        return 0.0;
    }

    @Override
    public double getReducedCostPenaltyForDroneArc(int park, DroneSchedule schedule, int numDrones) {
        return 0.0;
    }
}