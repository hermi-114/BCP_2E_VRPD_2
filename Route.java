import java.util.List;

public class Route {
    public final List<Node> sequence;
    public double totalTime;
    
    public Route(List<Node> sequence) {
        this.sequence = sequence;
        totalTime = -1;
    }
}
