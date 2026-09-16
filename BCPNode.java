import java.util.*;

public class BCPNode {
    public List<Route> forcedRoutes  = new ArrayList<>();
    public Set<String> forbiddenSigs = new HashSet<>();
    public int depth = 0;
}