public class BranchDecision {
    public final BranchCandidate candidate;
    public final boolean upperBound;   // true → ≤, false → ≥
    public final double  rhs;

    public BranchDecision(BranchCandidate candidate, boolean upperBound, double rhs) {
        this.candidate = candidate;
        this.upperBound = upperBound;
        this.rhs = rhs;
    }

    @Override
    public String toString() {
        return candidate + (upperBound ? " ≤ " : " ≥ ") + rhs;
    }
}