package MapMatch.shortpath;

import MapMatch.pathMatch.PathMatcher;
import MapMatch.pathMatch.PathRoadMap;
import com.bmwcarit.barefoot.matcher.Matcher;
import com.bmwcarit.barefoot.matcher.MatcherCandidate;
import com.bmwcarit.barefoot.matcher.MatcherSample;
import com.bmwcarit.barefoot.matcher.Minset;
import com.bmwcarit.barefoot.road.Heading;
import com.bmwcarit.barefoot.roadmap.*;
import com.bmwcarit.barefoot.scheduler.StaticScheduler;
import com.bmwcarit.barefoot.scheduler.Task;
import com.bmwcarit.barefoot.spatial.Geography;
import com.bmwcarit.barefoot.spatial.SpatialOperator;
import com.bmwcarit.barefoot.topology.Cost;
import com.bmwcarit.barefoot.topology.Dijkstra;
import com.bmwcarit.barefoot.topology.Router;
import com.bmwcarit.barefoot.util.Tuple;
import com.esri.core.geometry.Point;
import utils.CommonUtils;

import java.io.Serializable;
import java.util.*;
import java.util.concurrent.PriorityBlockingQueue;

/**
 * 滑动窗口式连续局部最短路径匹配。获取窗口起点集到终点集间所有最短路径，评价每条路径的权重（误差距离 * 曲线相似度权重），并对疑似路径进行非最短路径行驶检测
 */
public class CNSPMatcher extends Matcher implements Serializable {

    /**
     * 滑动窗口相关参数
     */
    private final int TRAJ_LEN = 600; 
    private final double OVERLAP_LEN = TRAJ_LEN / 2; 
    private final double PRE_LEN = TRAJ_LEN - OVERLAP_LEN; 
    private final int WINDOW_PATH_NUM = 4; 

    /**
     * NSP检测相关参数
     */
    private static final double NSP_MIN_LEN = 35;
    private static final int NSP_POINT_NUM = 2; 

    /**
     * 其他参数
     */
    public final static int maxSpeed = 30; 
    private static final int MAX_RETRY_TIMES = 4; 
    private static final int maxAzimuth = 90; 

    /**
     * 功能开关
     */
    private boolean useCurveSimilarity = true;

//----------------------------------------------------------------------------------------------------------------------
    //使用动态误差分布参数时重写以下方法
    double getRadius(Point point) {
        return getMaxRadius();
    }

    double getAdValue(Point point, double x) {
        return 18*x;
    }

    double getDistScore(Point point, double dist) {
        return 1 / sqrt_2pi_sig2 * Math.exp((-1) * dist * dist / (2 * sig2));
    }

//----------------------------------------------------------------------------------------------------------------------


    public CNSPMatcher(RoadMap map, Router<Road, RoadPoint> router, Cost<Road> cost, SpatialOperator spatial) {
        super(map, router, cost, spatial);
    }

    public CNSPMatcher(RoadMap map) {
        super(map, new Dijkstra<Road, RoadPoint>(), new TimePriority(), new Geography());
    }

    public CNSPMatcher(RoadMap map, Integer maxRadius) {
        super(map, new Dijkstra<Road, RoadPoint>(), new TimePriority(), new Geography());
        this.setMaxRadius(maxRadius);
    }

    @Override
    public List<MatcherCandidate> getMatchCandidates(List<MatcherSample> samples) {
        
        List<MatcherSample> trajectory = new ArrayList<>(); 
        int windowLeft = 0;int[] nextStart = new int[1];int temp = -1;
        MatcherSample previousSample = samples.get(0);
        Set<SPCandi> candidates_ = getCandidates(previousSample);
        
        while (candidates_.isEmpty() && windowLeft < samples.size()-2) {
            previousSample = samples.get(++windowLeft);
            candidates_ = getCandidates(previousSample);
        }
        if (candidates_.isEmpty()) return new ArrayList<>();
        trajectory.add(previousSample);
        int windowRight = windowLeft, start = windowLeft;
        double curWindowLen = 0.0;
        
        HashMap<SPCandi, Double> nextWindowStarts = new HashMap<>();
        for (SPCandi spCandi : candidates_) {
            nextWindowStarts.put(spCandi, 0.0);
        }
        boolean hasSetNextStart = false;
        List<Tuple<List<SPCandi>, Double>> sequenceList = null, lastMatchSequenceList = null;
        int retry_times = 0;
        for (int i = windowLeft+1;i < samples.size();i++) {
            MatcherSample sample = samples.get(i);
            trajectory.add(sample);
            windowRight++;
            double distance = spatial.distance(previousSample.point(), sample.point()); 
            sample.curLen = previousSample.curLen + distance;
            if (useCurveSimilarity) {
                sample.azimuth = CommonUtils.calculateAzimuth(sample.point(), previousSample.point()); 
            }
            if (!hasSetNextStart && curWindowLen+distance >= PRE_LEN) {
                temp = i - windowLeft;hasSetNextStart = true; 
            }
            curWindowLen += distance;
            previousSample = sample;
            if (curWindowLen >= TRAJ_LEN) { 
                nextStart[0] = temp;
                sequenceList = getTrajectoryMatch(trajectory, nextWindowStarts); 
                
                if (sequenceList.isEmpty()) {

                    if (++retry_times > MAX_RETRY_TIMES) return getResult(lastMatchSequenceList); 
                    continue;
                }
                lastMatchSequenceList = sequenceList;
                retry_times = 0;
                nextWindowStarts = getNextWindowStarts(sequenceList, nextStart);
                if (nextStart[0] <= 0 || nextStart[0] >= trajectory.size())
                    throw new RuntimeException("nextWindowStarts为空！");
                windowLeft += nextStart[0]; 
                hasSetNextStart = false;
                trajectory = new ArrayList<>(samples.subList(windowLeft, windowRight+1)); 
                curWindowLen = samples.get(windowRight).curLen - samples.get(windowLeft).curLen;
            }
        }

        
        if (sequenceList == null || sequenceList.isEmpty() || sequenceList.get(0).one().get(sequenceList.get(0).one().size()-1).sample() != previousSample) {
            while (windowLeft > start && curWindowLen < TRAJ_LEN) { 
                curWindowLen = samples.get(samples.size()-1).curLen - samples.get(--windowLeft).curLen;
                nextStart[0]--;
            }
            if (sequenceList != null && !sequenceList.isEmpty()) {
                nextWindowStarts = getNextWindowStarts(sequenceList, nextStart); 
            }
            
            candidates_ = getCandidates(samples.get(windowRight));
            while (candidates_.isEmpty() && windowRight > windowLeft-1) {
                candidates_ = getCandidates(samples.get(--windowRight));
            }
            trajectory = new ArrayList<>(samples.subList(windowLeft, windowRight+1));

            
            sequenceList = getTrajectoryMatch(trajectory, nextWindowStarts); 
            
            retry_times = 0;
            while (sequenceList.isEmpty() && trajectory.size() >= 2
                    && ++retry_times <= MAX_RETRY_TIMES
            ) {
                trajectory.remove(trajectory.size()-1);
                sequenceList = getTrajectoryMatch(trajectory, nextWindowStarts);
            }
            if (!sequenceList.isEmpty()) lastMatchSequenceList = sequenceList;
        }

        return getResult(lastMatchSequenceList);
    }

    private List<MatcherCandidate> getResult(List<Tuple<List<SPCandi>, Double>> sequenceList) {
        List<MatcherCandidate> result = new ArrayList<>();
        
        if (sequenceList != null && !sequenceList.isEmpty()) {
            Tuple<List<SPCandi>, Double> maxScorePath = sequenceList.get(0);
            for (Tuple<List<SPCandi>, Double> t : sequenceList) {
                if (t.two() > maxScorePath.two())
                    maxScorePath = t;
            }
            MatcherCandidate candi = maxScorePath.one().get(maxScorePath.one().size() - 1);
            while (candi != null) { 
                result.add(candi);
                candi = candi.predecessor();
            }
            Collections.reverse(result);
        }
        return result;
    }


    /**
     * sequence的中间匹配点是从PathMap中获取的，不具备完整地图信息。此方法从完整地图中获取下一个窗口的起始候选点集
     */
    public HashMap<SPCandi, Double> getNextWindowStarts(List<Tuple<List<SPCandi>, Double>> sequenceList, int[] nextStart) {
        HashMap<SPCandi, Double> nextWindowStarts = new HashMap<>();
        int len = sequenceList.get(0).one().size(); 
        boolean moveLeft = true;int left = nextStart[0], right = nextStart[0]; 
        
        while (nextWindowStarts.isEmpty() && nextStart[0] >= 0 && nextStart[0] < len) { 
            for (Tuple<List<SPCandi>, Double> sequence : sequenceList) {
                SPCandi c = sequence.one().get(nextStart[0]);
                
                if (c != null && (!nextWindowStarts.containsKey(c) || nextWindowStarts.get(c) < sequence.two())) {
                    Road road = map.edges.get(c.point().edge().id());
                    RoadPoint point = new RoadPoint(road, c.point().fraction());
                    SPCandi candi = new SPCandi(point, c.sample());
                    c.accumScore = sequence.two();
                    copyCandiInfo(candi, c);
                    nextWindowStarts.put(candi, candi.accumScore);
                }
            }
            
            if (nextWindowStarts.isEmpty()) {
                if (moveLeft) nextStart[0] = --left; else nextStart[0] = ++right;
                moveLeft = !moveLeft;
            }
        }
        return nextWindowStarts;
    }

    /**
     * 将c2的信息拷贝到c1中
     */
    private void copyCandiInfo(SPCandi c1, SPCandi c2) {
        c1.score = c2.score;
        c1.accumScore = c2.accumScore;
        c1.isCompCandi = c2.isCompCandi;
        
        c1.predecessor(c2.predecessor());
        c1.transition(c2.transition());
    }

    public List<Tuple<List<SPCandi>, Double>> getTrajectoryMatch(final List<MatcherSample> trajectory, final HashMap<SPCandi, Double> nextWindowStarts) {

        List<Tuple<List<SPCandi>, Double>> sequenceList = new ArrayList<>();
        
        if (trajectory.size() <= 1) {
            Map.Entry<SPCandi, Double> e = null;
            for (Map.Entry<SPCandi, Double> entry : nextWindowStarts.entrySet())
                if (e == null || e.getValue() < entry.getValue() || (e.getValue().equals(entry.getValue()) && e.getKey().score > entry.getKey().score)) e = entry;
            Tuple<List<SPCandi>, Double> tuple = new Tuple<>(Arrays.asList(e.getKey()), e.getValue());
            sequenceList.add(tuple);
            return sequenceList;
        }
        
        
        MatcherSample startSample = trajectory.get(0);
        MatcherSample endSample = trajectory.get(trajectory.size()-1);

        Set<SPCandi> endCandidates = getCandidates(trajectory.get(trajectory.size()-1)); 
        
        
        StaticScheduler.InlineScheduler scheduler = StaticScheduler.scheduler();
        double sTime = (double) (endSample.time() - startSample.time()) / 1000; 
        final double bound = Math.max(100, sTime * maxSpeed); 
        PriorityBlockingQueue<Tuple<List<SPCandi>, Double>> matchSequenceQueue = new PriorityBlockingQueue<>(10, (o1, o2) -> Double.compare(o2.two(), o1.two()));
        
        for (final SPCandi startCandi : nextWindowStarts.keySet()) {
            for (final SPCandi endCandi : endCandidates) {
                scheduler.spawn(new Task() {
                    @Override
                    public void run() {
                        
                        
                        RoadPoint startPoint = startCandi.point();RoadPoint endPoint = endCandi.point();
                        List<Road> edges = router.route(startPoint, endPoint, CostInstance.DISTANCE, CostInstance.DISTANCE, bound);
                        if (edges != null && !edges.isEmpty()) {

                            PathRoadMap pathMap = new PathRoadMap(edges, getMaxRadius()); 
                            pathMap.construct();
                            
                            MatcherSample startCandiSample = new MatcherSample(startSample.time(), new Point(startPoint.geometry().getX(), startPoint.geometry().getY()));startCandiSample.curLen = startSample.curLen;
                            MatcherSample endCandiSample = new MatcherSample(endSample.time(), new Point(endPoint.geometry().getX(), endPoint.geometry().getY()));endCandiSample.curLen = endSample.curLen;
                            List<MatcherSample> ss = new ArrayList<>(); 
                            ss.add(startCandiSample);ss.addAll(trajectory.subList(1, trajectory.size()-1));ss.add(endCandiSample);
                            
                            SPCandi sCandi = new SPCandi(startCandi.point(), startCandi.sample());copyCandiInfo(sCandi, startCandi);
                            SPCandi eCandi = new SPCandi(endCandi.point(), endCandi.sample());copyCandiInfo(eCandi, endCandi);
                            List<SPCandi> matchSequence = PathMatcher.getMatchWithFixedStartAndEnd(ss, pathMap, sCandi, eCandi);
                            if (matchSequence != null) {
                                
                                double score = calculateScore(matchSequence); 

                                    matchSequenceQueue.add(new Tuple<>(matchSequence, score + startCandi.accumScore)); 
                            }
                        }

                    }
                });
            }
        }
        if (!scheduler.sync()) {
            throw new RuntimeException();
        }

        
        for (int i = 1; i <= WINDOW_PATH_NUM && !matchSequenceQueue.isEmpty(); i++) {
            Tuple<List<SPCandi>, Double> sequence = matchSequenceQueue.poll();
            
            double trajLen = sequence.one().get(sequence.one().size() - 1).sample().curLen - sequence.one().get(0).sample().curLen;
            if (trajLen >= NSP_MIN_LEN && sequence.one().size() >= NSP_POINT_NUM+2) {
                sequence = checkAndMatchNonSpTrajectory(sequence, trajectory);
            }
            sequenceList.add(sequence);
        }
        return sequenceList;
    }

    /**
     * 检测是否非局部最短路径行驶，若是则通过在误差最大处分段递归匹配，获取sequence所使用的起点和终点间的更优路径
     */
    public Tuple<List<SPCandi>, Double> checkAndMatchNonSpTrajectory(Tuple<List<SPCandi>, Double> sequence, List<MatcherSample> trajectory) {
        final List<SPCandi> list = sequence.one();
        int index = checkNonSpTrajectory(list);
        if (index != -1) {
            

            
            HashMap<SPCandi, Double> leftWindowStarts = new HashMap<>();leftWindowStarts.put(list.get(0), list.get(0).accumScore);
            ArrayList<MatcherSample> leftHalf = new ArrayList<>(trajectory.subList(0, index + 1));

            List<Tuple<List<SPCandi>, Double>> leftResult = getTrajectoryMatch(leftHalf, leftWindowStarts, null, sequence.one().subList(0, index + 1)); 
            if (leftResult.isEmpty()) return sequence;
            
            HashMap<SPCandi, Double> rightWindowStarts = getNextWindowStarts(leftResult, new int[]{leftHalf.size()-1});
            ArrayList<MatcherSample> rightHalf = new ArrayList<>(trajectory.subList(index, list.size()));

            Set<SPCandi> endCandidates = new HashSet<>();endCandidates.add(list.get(list.size()-1)); 
            List<Tuple<List<SPCandi>, Double>> rightResult = getTrajectoryMatch(rightHalf, rightWindowStarts, endCandidates, sequence.one().subList(index, list.size())); 
            if (rightResult.isEmpty()) return sequence;
            
            SPCandi oriEnd = list.get(list.size() - 1);
            Tuple<List<SPCandi>, Double> maxScoreRightPath = null;
            for (Tuple<List<SPCandi>, Double> t : rightResult) {
                SPCandi rightLast = t.one().get(t.one().size() - 1);
                if (rightLast != null && rightLast.equals(oriEnd)) {
                    if (maxScoreRightPath == null) maxScoreRightPath = t;
                    else {
                        if (t.two() > maxScoreRightPath.two())
                            maxScoreRightPath = t;
                    }
                }
            }
            if (maxScoreRightPath == null) return sequence;
            
            if (maxScoreRightPath.two() > sequence.two()) {
                
                SPCandi rightStart = maxScoreRightPath.one().get(0);
                List<SPCandi> maxScoreLeftPath = null;
                for (Tuple<List<SPCandi>, Double> tuple : leftResult) {
                    
                    SPCandi leftLast = tuple.one().get(tuple.one().size() - 1);
                    if (leftLast != null && leftLast.equals(rightStart)
                            && leftLast.predecessor() == rightStart.predecessor() && tuple.two() == rightStart.accumScore)
                        maxScoreLeftPath = tuple.one();
                }
                maxScoreLeftPath.remove(maxScoreLeftPath.size()-1);
                List<SPCandi> resultSequence = new ArrayList<>(list.size());
                resultSequence.addAll(maxScoreLeftPath);
                resultSequence.addAll(maxScoreRightPath.one());
                if (isValidNonSpSequence(sequence.one(), resultSequence)) 
                    sequence = new Tuple<>(resultSequence, resultSequence.get(0).accumScore + calculateScore(resultSequence)); 
            }
        }
        return sequence;
    }

    /**
     * 在做拆分时，若拆分后的结果路径只是在中间增加了多对来回往返的道路，并且这些中间点原来的匹配误差不超过radius，则视为无效拆分
     * 避免在T形或十字路口的绕路
     */
    public boolean isValidNonSpSequence(List<SPCandi> originalSequence, List<SPCandi> resultSequence) {
        if (resultSequence == null || resultSequence.size() != originalSequence.size()) return false;
        int startIndex = 0; 
        while (startIndex < originalSequence.size() && originalSequence.get(startIndex).equals(resultSequence.get(startIndex))) {
            startIndex++;
        }
        int endIndex = originalSequence.size()-1; 
        while (endIndex >= startIndex && originalSequence.get(endIndex).equals(resultSequence.get(endIndex))) {
            endIndex--;
        }
        endIndex++;
        if (startIndex >= endIndex) return true;
        HashMap<Long, Set<Heading>> resultPath = new HashMap<>(); 
        
        for (Road road : resultSequence.get(startIndex).transition().route().path()) { 
            if (resultPath.containsKey(road.base().id())) {
                resultPath.get(road.base().id()).add(road.heading());
            } else {
                HashSet<Heading> set = new HashSet<>();set.add(road.heading());
                resultPath.put(road.base().id(), set);
            }

        }
        for (Road road : originalSequence.get(startIndex).transition().route().path()) {
            if (resultPath.containsKey(road.base().id())) {
                resultPath.get(road.base().id()).remove(road.heading());
            }
        }
        for (Road road : resultSequence.get(endIndex).transition().route().path()) {
            if (resultPath.containsKey(road.base().id())) {
                resultPath.get(road.base().id()).add(road.heading());
            } else {
                HashSet<Heading> set = new HashSet<>();set.add(road.heading());
                resultPath.put(road.base().id(), set);
            }
        }
        for (Road road : originalSequence.get(endIndex).transition().route().path()) {
            if (resultPath.containsKey(road.base().id())) {
                resultPath.get(road.base().id()).remove(road.heading());
            }
        }
        
        for (int i = startIndex+1; i < endIndex; i++) {
            for (Road road : resultSequence.get(i).transition().route().path()) {
                if (resultPath.containsKey(road.base().id())) {
                    resultPath.get(road.base().id()).add(road.heading());
                } else {
                    HashSet<Heading> set = new HashSet<>();set.add(road.heading());
                    resultPath.put(road.base().id(), set);
                }
            }
        }
        if (resultPath.isEmpty()) return true;
        for (Set<Heading> headings : resultPath.values()) { 
            if (headings.size() == 1) return true;
        }
        
        for (int i = startIndex+1; i <= endIndex; i++) {
            if (originalSequence.get(i).score > getRadius(originalSequence.get(i).point().geometry()))
                return true;
        }
        return false;
    }


    public int checkNonSpTrajectory(List<SPCandi> list) {
        int count = 0;
        double maxScore = Double.MIN_VALUE;
        int maxScoreIndex = -1, leftIndex = -1, rightIndex = -1;
        boolean foundSequence = false;
        for (int i = 1; i < list.size()-1; i++) {
            if (list.get(i) == null) continue;
            if (list.get(i).score > getAdValue(list.get(i).point().geometry(), 0.85)) {
                if (count == 0) {
                    leftIndex = i;rightIndex = i;
                } else rightIndex = i;
                count++;
                
                if (list.get(i).score > maxScore) {
                    maxScore = list.get(i).score;
                    maxScoreIndex = i;
                }
                
                if (count >= NSP_POINT_NUM && maxScore > getAdValue(list.get(i).point().geometry(), 0.95)) {
                    foundSequence = true;
                }
            } else {
                
                if (leftIndex > 0) leftIndex--;if (rightIndex < list.size()-1) rightIndex++; 
                if (foundSequence && (list.get(rightIndex).sample().curLen - list.get(leftIndex).sample().curLen > NSP_MIN_LEN)) {
                    return maxScoreIndex;
                }
                
                count = 0;
                maxScore = Double.MIN_VALUE;
                maxScoreIndex = -1;
                leftIndex = -1;rightIndex = -1;
                foundSequence = false;
            }
        }

        
        if (leftIndex > 0) leftIndex--;if (rightIndex < list.size()-1) rightIndex++; 
        return foundSequence && (list.get(rightIndex).sample().curLen - list.get(leftIndex).sample().curLen > NSP_MIN_LEN)
                
                && !list.get(maxScoreIndex).sample().point().equals(list.get(0).sample().point())
                && !list.get(maxScoreIndex).sample().point().equals(list.get(list.size()-1).sample().point())
                ? maxScoreIndex : -1;
    }

    public double calculateScore(List<SPCandi> matchSequence) {
        double sumScore = 0.0;
        for (MatcherCandidate candidate : matchSequence) {
            if (candidate != null) {
                double emission = getDistScore(candidate.point().geometry(), candidate.score);
                sumScore += emission;
            }
        }
        if (useCurveSimilarity) {
            double sumRlb = 0.0;
            for (int i = 1; i < matchSequence.size(); i++) {
                MatcherCandidate candidate = matchSequence.get(i);
                double ca = CommonUtils.calculateAzimuth(candidate.point().geometry(), matchSequence.get(i-1).point().geometry()); 
                double d = CommonUtils.azimuthDifference(ca, candidate.sample().azimuth); 
                double rlb = d > maxAzimuth ? 0 : 1 - d/maxAzimuth; 
                sumRlb += rlb; 
            }
            sumScore *= sumRlb / (matchSequence.size()-1); 
        }
        return sumScore;
    }

    protected Set<SPCandi> getCandidates(MatcherSample sample) {
        
        Set<RoadPoint> points_ = map.spatial().radius(sample.point(), getRadius(sample.point()));
        
        Set<RoadPoint> points = new HashSet<>(Minset.minimize(points_));
        Set<SPCandi> candidates = new HashSet<>();
        for (RoadPoint point : points) {
            SPCandi candidate = new SPCandi(point, sample);
            candidate.score = spatial.distance(sample.point(), new Point(point.geometry().getX(), point.geometry().getY()));
            candidates.add(candidate);
        }

        return candidates;
    }

    /**
     * 解决递归拆分时指数级拆分问题
     * 若拆分后最优路径仍然匹配出了相同的结果，则直接返回。避免一条横线道路上方一条斜线异常轨迹造成的指数级拆分
     */
    public List<Tuple<List<SPCandi>, Double>> getTrajectoryMatch(final List<MatcherSample> trajectory, final HashMap<SPCandi, Double> nextWindowStarts, Set<SPCandi> endCandidates, List<SPCandi> originalMatch) {

        List<Tuple<List<SPCandi>, Double>> sequenceList = new ArrayList<>();
        
        if (trajectory.size() <= 1) {
            Map.Entry<SPCandi, Double> e = null;
            for (Map.Entry<SPCandi, Double> entry : nextWindowStarts.entrySet())
                if (e == null || e.getValue() < entry.getValue() || (e.getValue().equals(entry.getValue()) && e.getKey().score > entry.getKey().score)) e = entry;
            Tuple<List<SPCandi>, Double> tuple = new Tuple<>(Arrays.asList(e.getKey()), e.getValue());
            sequenceList.add(tuple);
            return sequenceList;
        }
        
        
        MatcherSample startSample = trajectory.get(0);
        MatcherSample endSample = trajectory.get(trajectory.size()-1);
        if (endCandidates == null)
            endCandidates = getCandidates(trajectory.get(trajectory.size()-1)); 
        double sTime = (double) (endSample.time() - startSample.time()) / 1000; 
        
        
        StaticScheduler.InlineScheduler scheduler = StaticScheduler.scheduler();
        final double bound = Math.max(100, sTime * maxSpeed); 
        PriorityBlockingQueue<Tuple<List<SPCandi>, Double>> matchSequenceQueue = new PriorityBlockingQueue<>(10, (o1, o2) -> Double.compare(o2.two(), o1.two()));
        
        for (final SPCandi startCandi : nextWindowStarts.keySet()) {
            for (final SPCandi endCandi : endCandidates) {
                scheduler.spawn(new Task() {
                    @Override
                    public void run() {
                        
                        
                        RoadPoint startPoint = startCandi.point();RoadPoint endPoint = endCandi.point();
                        List<Road> edges = router.route(startPoint, endPoint, CostInstance.DISTANCE, CostInstance.DISTANCE, bound);
                        if (edges != null && !edges.isEmpty()) { 
                            



                            PathRoadMap pathMap = new PathRoadMap(edges, getMaxRadius()); 
                            pathMap.construct();
                            
                            MatcherSample startCandiSample = new MatcherSample(startSample.time(), new Point(startPoint.geometry().getX(), startPoint.geometry().getY()));startCandiSample.curLen = startSample.curLen;
                            MatcherSample endCandiSample = new MatcherSample(endSample.time(), new Point(endPoint.geometry().getX(), endPoint.geometry().getY()));endCandiSample.curLen = endSample.curLen;
                            List<MatcherSample> ss = new ArrayList<>(); 
                            ss.add(startCandiSample);ss.addAll(trajectory.subList(1, trajectory.size()-1));ss.add(endCandiSample);
                            
                            SPCandi sCandi = new SPCandi(startCandi.point(), startCandi.sample()); copyCandiInfo(sCandi, startCandi);
                            SPCandi eCandi = new SPCandi(endCandi.point(), endCandi.sample()); copyCandiInfo(eCandi, endCandi);
                            List<SPCandi> matchSequence = PathMatcher.getMatchWithFixedStartAndEnd(ss, pathMap, sCandi, eCandi);
                            if (matchSequence != null) {
                                
                                double score = calculateScore(matchSequence); 

                                matchSequenceQueue.add(new Tuple<>(matchSequence, score + startCandi.accumScore)); 
                            }
                        }

                    }
                });
            }
        }
        if (!scheduler.sync()) {
            throw new RuntimeException();
        }

        
        for (int i = 1; i <= WINDOW_PATH_NUM / 2  && !matchSequenceQueue.isEmpty(); i++) { 
            Tuple<List<SPCandi>, Double> sequence = matchSequenceQueue.poll();
            if (i == 1) { 
                boolean eq = true;
                for (int j = 0; j < sequence.one().size(); j++) {
                    if (!sequence.one().get(j).equals(originalMatch.get(j))) {
                        eq = false;break;
                    }
                }
                if (eq) {
                    return sequenceList;
                }
            }
            
            double trajLen = sequence.one().get(sequence.one().size() - 1).sample().curLen - sequence.one().get(0).sample().curLen;
            if (trajLen >= NSP_MIN_LEN && sequence.one().size() >= NSP_POINT_NUM+2) {
                sequence = checkAndMatchNonSpTrajectory(sequence, trajectory);
            }
            sequenceList.add(sequence);
        }
        return sequenceList;
        
    }

}
