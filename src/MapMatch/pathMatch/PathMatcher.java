package MapMatch.pathMatch;

import MapMatch.shortpath.CNSPMatcher;
import MapMatch.shortpath.SPCandi;
import com.bmwcarit.barefoot.matcher.MatcherSample;
import com.bmwcarit.barefoot.matcher.MatcherTransition;
import com.bmwcarit.barefoot.matcher.Minset;
import com.bmwcarit.barefoot.roadmap.*;
import com.bmwcarit.barefoot.spatial.Geography;
import com.bmwcarit.barefoot.spatial.SpatialOperator;
import com.bmwcarit.barefoot.topology.Dijkstra;
import com.bmwcarit.barefoot.topology.Router;
import com.bmwcarit.barefoot.util.Tuple;

import java.util.*;
import java.util.function.Function;
import java.util.stream.Collectors;

/**
 * 单路径匹配
 */
public class PathMatcher {

    static final Router<Road, RoadPoint> router = new Dijkstra<>();
    static final SpatialOperator spatial = new Geography();

    public static List<SPCandi> getMatchWithFixedStartAndEnd(List<MatcherSample> samples, PathRoadMap pathMap, SPCandi start, SPCandi end) {
        Set<Tuple<SPCandi, Double>> lastCandis = getLastCandis(samples, pathMap, start, end);
        if (lastCandis == null || lastCandis.isEmpty()) return null;
        return findSequence(samples, lastCandis, start, end);
    }

    /**
     * 返回匹配完成后候选图的终点候选点集
     */
    public static Set<Tuple<SPCandi, Double>> getLastCandis(List<MatcherSample> samples, PathRoadMap pathMap, SPCandi start, SPCandi end) {
        List<Tuple<MatcherSample, Set<Tuple<SPCandi, Double>>>> candiGraph = new ArrayList<>();
        
        MatcherSample previousSample = samples.get(0);
        Set<SPCandi> firstCandidates = getFixedCandidate(previousSample, pathMap, start);
        Set<Tuple<SPCandi, Double>> predecessors = new HashSet<>();
        for (SPCandi candidate : firstCandidates) {
            predecessors.add(new Tuple<>(candidate, 0.0)); 
        }
        if (predecessors.isEmpty()) {
            throw new NullPointerException();
        }
        candiGraph.add(new Tuple<>(previousSample, predecessors));
        
        for (int i = 1;i < samples.size();i++) {
            MatcherSample sample = samples.get(i);
            Set<SPCandi> candidates = null;
            if (i < samples.size()-1)
                candidates = getCandidates(sample, pathMap);
            else  { 
                candidates = getFixedCandidate(sample, pathMap, end);
                if (candidates.isEmpty()) throw new NullPointerException();
            }
            
            
            if (candidates.size() == 0) {
                continue;
            }
            Set<Tuple<SPCandi, Double>> result = execute(predecessors, candidates, previousSample, sample);
            if (!result.isEmpty()) {
                candiGraph.add(new Tuple<>(sample, result)); 
                predecessors = result;
                previousSample = sample;
            } else if (i == samples.size()-1) return null; 

        }
        return candiGraph.get(candiGraph.size()-1).two();
    }


    private static Set<SPCandi> getCandidates(MatcherSample sample, PathRoadMap pathMap) {
        
        Set<RoadPoint> points_ = pathMap.spatial().radius(sample.point(), pathMap.radius);
        
        HashSet<RoadPoint> points = new HashSet<>(Minset.minimize(points_));
        Set<SPCandi> candidates = new HashSet<>();
        for (RoadPoint point : points) {
            SPCandi candidate = new SPCandi(point, sample);
            candidate.score = spatial.distance(sample.point(), candidate.point().geometry()); 
            candidates.add(candidate);
        }
        return candidates;
    }




    private static Set<SPCandi> getFixedCandidate(MatcherSample sample, PathRoadMap pathMap, SPCandi fixedCandi) {
        Road road = pathMap.edges.get(fixedCandi.point().edge().id());
        RoadPoint point = new RoadPoint(road, fixedCandi.point().fraction());
        SPCandi candi = new SPCandi(point, sample);
        candi.score = spatial.distance(sample.point(), candi.point().geometry()); 
        Set<SPCandi> candidates = new HashSet<>();
        candidates.add(candi);
        return candidates;
    }


    private static Set<Tuple<SPCandi, Double>> execute(Set<Tuple<SPCandi, Double>> predecessors, Set<SPCandi> candidates,
                                                                   MatcherSample previous, MatcherSample sample) {
        Set<Tuple<SPCandi, Double>> result = new HashSet<>();


        
        Map<SPCandi, Double> pointMap = new HashMap<>();
        Set<RoadPoint> points = new HashSet<>();
        for (SPCandi candidate : candidates) {
            points.add(candidate.point());
            pointMap.put(candidate, Double.MAX_VALUE);
        }

        
        double sTime = (double) (sample.time() - previous.time()) / 1000; 
        final double bound = Math.max(100, sTime * CNSPMatcher.maxSpeed);
        for (Tuple<SPCandi, Double> pre : predecessors) {
            
            Map<RoadPoint, List<Road>> routes =
                    router.route(pre.one().point(), points, CostInstance.DISTANCE, CostInstance.DISTANCE, bound);

            
            for (SPCandi candidate : pointMap.keySet()) {
                
                List<Road> edges = routes.get(candidate.point());
                if (edges == null) {
                    continue;
                }
                Route route = new Route(pre.one().point(), candidate.point(), edges);

                if (edges.size() >= 2) { 
                    if (edges.get(0).base().id() == edges.get(1).base().id()
                            && edges.get(0).id() != edges.get(1).id()) {
                        RoadPoint start = pre.one().point(), end = candidate.point();
                        if (edges.size() > 2) {
                            start = new RoadPoint(edges.get(1), 1 - start.fraction());
                            edges.remove(0);
                        } else {
                            
                            
                            
                            if (start.fraction() < 1 - end.fraction()) {
                                end = new RoadPoint(edges.get(0), Math.min(1d,
                                        1 - end.fraction() + (5d / edges.get(0).length())));
                                edges.remove(1);
                            } else {
                                start = new RoadPoint(edges.get(1), Math.max(0d, 1
                                        - start.fraction() - (5d / edges.get(1).length())));
                                edges.remove(0);
                            }
                        }
                        route = new Route(start, end, edges);
                    }
                }
                double curLength = pre.two() + route.length(); 
                double length = pointMap.get(candidate);
                
                if ((Math.abs(length - curLength) < 0.5 && candidate.predecessor().score > pre.one().score)
                            || (Math.abs(length-curLength) >= 0.5 && length > curLength)) {
                    pointMap.put(candidate, curLength); 
                    candidate.predecessor(pre.one()); 
                    candidate.transition(new MatcherTransition(route)); 
                }
            }
        }


        for (Map.Entry<SPCandi, Double> entry : pointMap.entrySet()) {
            if (entry.getValue() != Double.MAX_VALUE) {
                result.add(new Tuple<>(entry.getKey(), entry.getValue()));
            }
        }


        return result;
    }






    private static List<SPCandi> findSequence(List<MatcherSample> samples, Set<Tuple<SPCandi, Double>> lastCandis, SPCandi start, SPCandi end) {
        SPCandi lastCandi = lastCandis.iterator().next().one();
        
        List<SPCandi> candiList = new ArrayList<>();
        while (lastCandi != null) {
            candiList.add(lastCandi);
            lastCandi = (SPCandi)lastCandi.predecessor();
        }
        Collections.reverse(candiList);
        
        List<SPCandi> result = fillMissingWithCandis(samples, candiList);
        
        int i = 1;SPCandi snext = result.get(i);
        while (snext == null)
            snext = result.get(++i);
        snext.predecessor(start);
        result.set(0 , start);
        if (result.size() < samples.size() || result.get(samples.size()-1) == null)
            throw new NullPointerException();
        end.predecessor(result.get(samples.size()-1).predecessor());
        end.transition(result.get(samples.size()-1).transition()); 
        result.set(samples.size()-1, end);
        return result;
    }


    /**
     * 使用Null补全由于距离过远没有匹配到的点
     */
    private static List<SPCandi> fillMissingWithNull(List<MatcherSample> samples, List<SPCandi> candiList) {
        List<SPCandi> result = new ArrayList<>();
        int k = samples.size() - 1;
        SPCandi c = candiList.get(candiList.size()-1);
        while (c != null) {
            while (c.sample() != samples.get(k)) {
                result.add(null);k--; 
            }
            result.add(c);
            c = (SPCandi) c.predecessor();
            k--;
        }
        Collections.reverse(result);
        return result;
    }


    /**
     * 补全由于距离过远没有匹配到的点
     * 1.遍历samples列表，对于每个定位点，检查是否已存在于匹配轨迹中。
     * 2.对于未匹配到的定位点，找到其在samples列表中前后最近的已匹配定位点。
     * 3.计算未匹配定位点在两个已匹配定位点之间的位置比例。
     * 4.使用getPointAtFraction方法和计算出的比例，从两个已匹配定位点之间的路径上找到对应的RoadPoint。
     * 5.使用找到的RoadPoint和当前遍历到的未匹配定位点创建一个新的SPCandi对象。
     * 6.按照原始定位点列表的顺序，重新设置新创建的SPCandi对象的predecessor属性，以保持匹配轨迹的连续性。
     */
    private static List<SPCandi> fillMissingWithCandis(List<MatcherSample> samples, List<SPCandi> candiList) {
        List<SPCandi> filledCandiList = new ArrayList<>();
        Map<MatcherSample, SPCandi> sampleToCandiMap = candiList.stream()
                .collect(Collectors.toMap(SPCandi::sample, Function.identity()));

        for (int i = 0; i < samples.size(); i++) {
            MatcherSample currentSample = samples.get(i);
            if (!sampleToCandiMap.containsKey(currentSample)) { 
                
                SPCandi prevCandi = null;
                SPCandi nextCandi = null;
                for (int j = i - 1; j >= 0; j--) {
                    if (sampleToCandiMap.containsKey(samples.get(j))) {
                        prevCandi = sampleToCandiMap.get(samples.get(j));
                        break;
                    }
                }
                for (int k = i + 1; k < samples.size(); k++) {
                    if (sampleToCandiMap.containsKey(samples.get(k))) {
                        nextCandi = sampleToCandiMap.get(samples.get(k));
                        break;
                    }
                }

                if (prevCandi != null && nextCandi != null) {
                    
                    double frac = (currentSample.curLen - prevCandi.sample().curLen) / (nextCandi.sample().curLen - prevCandi.sample().curLen);
                    if (Double.isNaN(frac) || frac < 0) frac = 0;if (frac > 1) frac = 1;
                    Route route = nextCandi.transition().route(); 
                    
                    Tuple<Tuple<Route, Route>, RoadPoint> tuple = route.getPointAtFraction(frac);
                    
                    SPCandi newCandi = new SPCandi(tuple.two(), currentSample);
                    newCandi.isCompCandi = true; 
                    
                    newCandi.transition(new MatcherTransition(tuple.one().one()));
                    nextCandi.transition(new MatcherTransition(tuple.one().two()));
                    newCandi.score = spatial.distance(currentSample.point(), newCandi.point().geometry()); 
                    filledCandiList.add(newCandi);
                    sampleToCandiMap.put(currentSample, newCandi);
                }
            } else {
                filledCandiList.add(sampleToCandiMap.get(currentSample));
            }
        }

        
        for (int i = 1; i < filledCandiList.size(); i++) {
            filledCandiList.get(i).predecessor(filledCandiList.get(i - 1));
        }

        return filledCandiList;
    }


}
