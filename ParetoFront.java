import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;

public class ParetoFront {
    public List<DroneSchedule> nonDominatedSchedules;

    public ParetoFront() {
        this.nonDominatedSchedules = new ArrayList<>();
    }

    public void tryAddSchedule(DroneSchedule newSchedule) {

        if (newSchedule == null || 
            newSchedule.sequences == null || newSchedule.sequences.isEmpty() ||
            newSchedule.sequences.get(0) == null || newSchedule.sequences.get(0).isEmpty()
        ) return;

        Iterator<DroneSchedule> it = nonDominatedSchedules.iterator();

        while(it.hasNext()) {
            DroneSchedule schedule = it.next();

            if(schedule.dominates(newSchedule)) return;

            if(newSchedule.dominates(schedule)) it.remove();
        }

        nonDominatedSchedules.add(newSchedule);
    }
}
