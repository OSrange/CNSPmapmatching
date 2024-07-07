package MapMatch.shortpath;

import com.bmwcarit.barefoot.matcher.MatcherCandidate;
import com.bmwcarit.barefoot.matcher.MatcherSample;
import com.bmwcarit.barefoot.roadmap.Road;
import com.bmwcarit.barefoot.roadmap.RoadPoint;

import java.util.Objects;

public class SPCandi extends MatcherCandidate {

    public double accumScore;

    private int hash = 0; 

    public boolean isCompCandi = false; 

    public SPCandi(RoadPoint point) {
        super(point);
    }

    public SPCandi(RoadPoint point, MatcherSample sample) {
        super(point, sample);
    }

    public void setSample(MatcherSample sample) {
        this.sample = sample;
    }

    @Override
    public boolean equals(Object obj) {
        SPCandi c2 = (SPCandi) obj;
        Road edge1 = this.point().edge();
        Road edge2 = c2.point().edge();
        boolean sameRoad = edge1.base().refid() == edge2.base().refid() 
                && edge1.heading() == edge2.heading() 
                && edge1.source() == edge2.source() && edge1.target() == edge2.target(); 
        double abs = Math.abs(this.point().fraction() - c2.point().fraction()); 
        boolean sameFrac = abs < 0.0001;
        return sameRoad && sameFrac;
    }

    @Override
    public int hashCode() {
        if (hash != 0) return hash;
        Road edge = this.point().edge();
        hash = Objects.hash(edge.base().refid(), edge.heading(), edge.source(), edge.target(), String.format("%.4f", this.point().fraction()));
        return hash;
    }
}
