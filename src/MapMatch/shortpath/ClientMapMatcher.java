package MapMatch.shortpath;

import com.bmwcarit.barefoot.matcher.Matcher;
import com.bmwcarit.barefoot.matcher.MatcherCandidate;
import com.bmwcarit.barefoot.matcher.MatcherKState;
import com.bmwcarit.barefoot.matcher.MatcherSample;
import com.bmwcarit.barefoot.road.BfmapReader;
import com.bmwcarit.barefoot.roadmap.RoadMap;
import com.bmwcarit.barefoot.util.Tuple;

import java.io.Serializable;
import java.lang.reflect.InvocationTargetException;
import java.util.List;

/**
 * deploy-mode client模式下使用的可广播泛型Matcher。读取driver端文件系统路径下的map文件
 * @param <M>
 */
public class ClientMapMatcher<M extends Matcher> implements Serializable {

    private final RoadMap map;
    private int maxRadius = -1;
    private final Class<M> matcherClass;

    public ClientMapMatcher(RoadMap map, int maxRadius, Class<M> matcherClass) {
        this.map = map;
        this.maxRadius = maxRadius;
        this.matcherClass = matcherClass;
    }

    public ClientMapMatcher(String mapPath, Class<M> matcherClass) {
        this.map = RoadMap.Load(new BfmapReader(mapPath));
        this.matcherClass = matcherClass;
    }

    public ClientMapMatcher(String mapPath, int maxRadius, Class<M> matcherClass) {
        this.map = RoadMap.Load(new BfmapReader(mapPath));
        this.maxRadius = maxRadius;
        this.matcherClass = matcherClass;
    }

    
    
    private volatile M matcherInstance;

    /**
     * 双重检测锁（DoubleCheckLockSingleton, DCL），当分发到Executor后被使用时再进行初始化
     */
    private M getMatcher() throws NoSuchMethodException, InvocationTargetException, InstantiationException, IllegalAccessException {
        if (matcherInstance == null) {
            synchronized(this) {
                if (matcherInstance == null) { 
                    map.construct();
                    if (maxRadius == -1)
                        matcherInstance = matcherClass.getDeclaredConstructor(RoadMap.class).newInstance(map);
                    else
                        matcherInstance = matcherClass.getDeclaredConstructor(RoadMap.class, Integer.class).newInstance(map, maxRadius);
                }
            }
        }
        return matcherInstance;
    }

    public MatcherKState mmatch(List<MatcherSample> samples) {
        return mmatch(samples, 0, 0);
    }

    private MatcherKState mmatch(List<MatcherSample> samples, double minDistance, int minInterval) {
        try {
            return getMatcher().mmatch(samples, minDistance, minInterval);
        } catch (NoSuchMethodException | InvocationTargetException | InstantiationException | IllegalAccessException e) {
            e.printStackTrace();
        }
        return null;
    }

    public Tuple<List<MatcherSample>, List<MatcherCandidate>> getMatchSequence(List<MatcherSample> samples) {
        try {
            return getMatcher().getMatchSequence(samples);
        } catch (NoSuchMethodException | InvocationTargetException | InstantiationException | IllegalAccessException e) {
            e.printStackTrace();
        }
        return null;
    }

    public List<MatcherCandidate> getMatchCandidates(List<MatcherSample> samples) {
        try {
            return getMatcher().getMatchCandidates(samples);
        } catch (NoSuchMethodException | InvocationTargetException | InstantiationException | IllegalAccessException e) {
            e.printStackTrace();
        }
        return null;
    }
}
