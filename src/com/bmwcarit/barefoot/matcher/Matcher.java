/*
 * Copyright (C) 2015, BMW Car IT GmbH
 *
 * Author: Sebastian Mattheis <sebastian.mattheis@bmw-carit.de>
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may not use this file except
 * in compliance with the License. You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0 Unless required by applicable law or agreed to in
 * writing, software distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the License for the specific
 * language governing permissions and limitations under the License.
 */

package com.bmwcarit.barefoot.matcher;

import com.bmwcarit.barefoot.markov.Filter;
import com.bmwcarit.barefoot.markov.KState;
import com.bmwcarit.barefoot.road.Heading;
import com.bmwcarit.barefoot.roadmap.*;
import com.bmwcarit.barefoot.scheduler.StaticScheduler;
import com.bmwcarit.barefoot.scheduler.StaticScheduler.InlineScheduler;
import com.bmwcarit.barefoot.scheduler.Task;
import com.bmwcarit.barefoot.spatial.SpatialOperator;
import com.bmwcarit.barefoot.topology.Cost;
import com.bmwcarit.barefoot.topology.Router;
import com.bmwcarit.barefoot.util.Stopwatch;
import com.bmwcarit.barefoot.util.Tuple;
import com.esri.core.geometry.GeometryEngine;
import com.esri.core.geometry.WktExportFlags;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.text.SimpleDateFormat;
import java.util.*;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.atomic.AtomicInteger;

/**
 * Matcher filter for Hidden Markov Model (HMM) map matching. It is a HMM filter (@{link Filter})
 * and determines emission and transition probabilities for map matching with HMM.
 */
public class Matcher extends Filter<MatcherCandidate, MatcherTransition, MatcherSample> {
    private static final Logger logger = LoggerFactory.getLogger(Matcher.class);

    public final RoadMap map;
    public final Router<Road, RoadPoint> router;
    public final Cost<Road> cost;
    public final SpatialOperator spatial;

    public double sig2 = Math.pow(10d, 2);
    public double sigA = Math.pow(10d, 2);
    public double sqrt_2pi_sig2 = Math.sqrt(2d * Math.PI * sig2);
    public double sqrt_2pi_sigA = Math.sqrt(2d * Math.PI * sigA);
    public double lambda = 0d;
    public double radius = 200;
    public double distance = 15000;
    private boolean shortenTurns = true;

    /**
     * Creates a HMM map matching filter for some map, router, cost function, and spatial operator.
     *
     * @param map {@link RoadMap} object of the map to be matched to.
     * @param router {@link Router} object to be used for route estimation.
     * @param cost Cost function to be used for routing.
     * @param spatial Spatial operator for spatial calculations.
     */
    public Matcher(RoadMap map, Router<Road, RoadPoint> router, Cost<Road> cost,
            SpatialOperator spatial) {
        this.map = map;
        this.router = router;
        this.cost = cost;
        this.spatial = spatial;
    }

    /**
     * Gets standard deviation in meters of gaussian distribution that defines emission
     * probabilities.
     *
     * @return Standard deviation in meters of gaussian distribution that defines emission
     *         probabilities.
     */
    public double getSigma() {
        return Math.sqrt(this.sig2);
    }

    /**
     * Sets standard deviation in meters of gaussian distribution for defining emission
     * probabilities (default is 10 meters).
     *
     * @param sigma Standard deviation in meters of gaussian distribution for defining emission
     *        probabilities (default is 10 meters).
     */
    public void setSigma(double sigma) {
        this.sig2 = Math.pow(sigma, 2);
        this.sqrt_2pi_sig2 = Math.sqrt(2d * Math.PI * sig2);
    }

    /**
     * Gets lambda parameter of negative exponential distribution defining transition probabilities.
     *
     * @return Lambda parameter of negative exponential distribution defining transition
     *         probabilities.
     */
    public double getLambda() {
        return this.lambda;
    }

    /**
     * Sets lambda parameter of negative exponential distribution defining transition probabilities
     * (default is 0.0). It uses adaptive parameterization, if lambda is set to 0.0.
     *
     * @param lambda Lambda parameter of negative exponential distribution defining transition
     *        probabilities.
     */
    public void setLambda(double lambda) {
        this.lambda = lambda;
    }

    /**
     * Gets maximum radius for candidate selection in meters.
     *
     * @return Maximum radius for candidate selection in meters.
     */
    public double getMaxRadius() {
        return this.radius;
    }

    /**
     * Sets maximum radius for candidate selection in meters (default is 200 meters).
     *
     * @param radius Maximum radius for candidate selection in meters.
     */
    public void setMaxRadius(double radius) {
        this.radius = radius;
    }

    /**
     * Gets maximum transition distance in meters.
     *
     * @return Maximum transition distance in meters.
     */
    public double getMaxDistance() {
        return this.distance;
    }

    /**
     * Sets maximum transition distance in meters (default is 15000 meters).
     *
     * @param distance Maximum transition distance in meters.
     */
    public void setMaxDistance(double distance) {
        this.distance = distance;
    }

    /**
     * Gets option of shorten turns.
     *
     * @return Option of shorten turns.
     */
    public boolean shortenTurns() {
        return this.shortenTurns;
    }

    /**
     * Sets option to shorten turns (default true).
     *
     * @param shortenTurns Option to shorten turns.
     */
    public void shortenTurns(boolean shortenTurns) {
        this.shortenTurns = shortenTurns;
    }

    @Override
    protected Set<Tuple<MatcherCandidate, Double>> candidates(Set<MatcherCandidate> predecessors,
            MatcherSample sample) {
        if (logger.isTraceEnabled()) {
            logger.trace("finding candidates for sample {} {}",
                    new SimpleDateFormat("yyyy-MM-dd HH:mm:ssZ").format(sample.time()),
                    GeometryEngine.geometryToWkt(sample.point(), WktExportFlags.wktExportPoint));
        }


//        Stopwatch sw = new Stopwatch();
//        sw.start();
        //选出指定半径内sample的候选道路点集
        Set<RoadPoint> points_ = map.spatial().radius(sample.point(), radius);
//        sw.stop();
////        System.out.println("选择候选道路耗时：" + sw.ms());
//        TimeStatistic.statGetRoadTime(sw.us(), 1);
        Set<RoadPoint> points = new HashSet<>(Minset.minimize(points_));

        Map<Long, RoadPoint> map = new HashMap<>();
        for (RoadPoint point : points) {
            map.put(point.edge().id(), point);
        }

        //找到和前一阶段候选点在同一条道路上的候选点，若距离接近则替换
        for (MatcherCandidate predecessor : predecessors) {
            RoadPoint point = map.get(predecessor.point().edge().id());
            if (point != null && point.edge() != null
                    && spatial.distance(point.geometry(),
                            predecessor.point().geometry()) < getSigma()
                    && ((point.edge().heading() == Heading.forward
                            && point.fraction() < predecessor.point().fraction())
                            || (point.edge().heading() == Heading.backward
                                    && point.fraction() > predecessor.point().fraction()))) {
                points.remove(point);
                points.add(predecessor.point());
            }
        }

        Set<Tuple<MatcherCandidate, Double>> candidates = new HashSet<>();
        logger.debug("{} ({}) candidates", points.size(), points_.size());

        //计算每个候选点的观测概率
        for (RoadPoint point : points) {
            double dz = spatial.distance(sample.point(), point.geometry());
            double emission = 1 / sqrt_2pi_sig2 * Math.exp((-1) * dz * dz / (2 * sig2));
            if (!Double.isNaN(sample.azimuth())) {
                double da = sample.azimuth() > point.azimuth()
                        ? Math.min(sample.azimuth() - point.azimuth(),
                                360 - (sample.azimuth() - point.azimuth()))
                        : Math.min(point.azimuth() - sample.azimuth(),
                                360 - (point.azimuth() - sample.azimuth()));
                emission *=
                        Math.max(1E-2, 1 / sqrt_2pi_sigA * Math.exp((-1) * da * da / (2 * sigA)));
            }

            //候选点关联样本点
            MatcherCandidate candidate = new MatcherCandidate(point, sample);
            candidates.add(new Tuple<>(candidate, emission));

            logger.trace("{} {} {}", candidate.id(), dz, emission);
        }

        return candidates;
    }

    @Override
    protected Tuple<MatcherTransition, Double> transition(
            Tuple<MatcherSample, MatcherCandidate> predecessor,
            Tuple<MatcherSample, MatcherCandidate> candidate) {

        return null;
    }

    @Override
    protected Map<MatcherCandidate, Map<MatcherCandidate, Tuple<MatcherTransition, Double>>> transitions(
            final Tuple<MatcherSample, Set<MatcherCandidate>> predecessors,
            final Tuple<MatcherSample, Set<MatcherCandidate>> candidates) {

        if (logger.isTraceEnabled()) {
            logger.trace("finding transitions for sample {} {} with {} x {} candidates",
                    new SimpleDateFormat("yyyy-MM-dd HH:mm:ssZ").format(candidates.one().time()),
                    GeometryEngine.geometryToWkt(candidates.one().point(),
                            WktExportFlags.wktExportPoint),
                    predecessors.two().size(), candidates.two().size());
        }

        Stopwatch sw = new Stopwatch();
        sw.start();

        final Set<RoadPoint> targets = new HashSet<>();
        for (MatcherCandidate candidate : candidates.two()) {
            targets.add(candidate.point());
        }

        final AtomicInteger count = new AtomicInteger();
        final Map<MatcherCandidate, Map<MatcherCandidate, Tuple<MatcherTransition, Double>>> transitions =
                new ConcurrentHashMap<>();
        //候选路径最大搜索长度：max(1km, min(15km, 耗时*100m/s))
        final double bound = Math.max(1000d, Math.min(distance,
                ((candidates.one().time() - predecessors.one().time()) / 1000) * 100));

        //自定义全局线程池调度器
        InlineScheduler scheduler = StaticScheduler.scheduler();
        for (final MatcherCandidate predecessor : predecessors.two()) {
            scheduler.spawn(new Task() {
                @Override
                public void run() {
                    Map<MatcherCandidate, Tuple<MatcherTransition, Double>> map = new HashMap<>();
//                    Stopwatch sw2 = new Stopwatch();
//                    sw2.start();
                    //针对每一个前置候选点，用迪杰斯特拉算法获取它到当前所有候选点的道路路径
                    Map<RoadPoint, List<Road>> routes =
                            router.route(predecessor.point(), targets, cost, new Distance(), bound); //router这里似乎没有依赖地图，而仅仅是基于point的连接去搜索
//                    sw2.stop();
////                    System.out.println("搜索一个前置候选点到当前所有候选点的道路路径耗时：" + sw.ms());
//                    TimeStatistic.statGetPathTime(sw2.us(), routes.size());

                    logger.trace("{} routes ({} ms)", routes.size(), sw.ms());

                    //遍历当前候选点集
                    for (MatcherCandidate candidate : candidates.two()) {
                        List<Road> edges = routes.get(candidate.point());

                        if (edges == null) {
                            continue;
                        }

                        //获取 前置候选点 -> 当前候选点 路由
                        Route route = new Route(predecessor.point(), candidate.point(), edges);

                        //压缩路由中起始两条道路掉头转弯的现象
                        if (shortenTurns() && edges.size() >= 2) {
                            //检测起始的两条道路是否为被拆分成两个方向的Road
                            if (edges.get(0).base().id() == edges.get(1).base().id()
                                    && edges.get(0).id() != edges.get(1).id()) {
                                RoadPoint start = predecessor.point(), end = candidate.point();
                                if (edges.size() > 2) { //如果道路数量大于2，则直接将起点压缩到第二条道路上
                                    start = new RoadPoint(edges.get(1), 1 - start.fraction());
                                    edges.remove(0);
                                } else { //如果道路数量只有2条。假设为先向左再向右
                                    // Here, additional cost of 5 meters are added to the route
                                    // length in order to penalize and avoid turns, e.g., at the end
                                    // of a trace.
                                    if (start.fraction() < 1 - end.fraction()) { //若end在start的左边，则直接将end压缩到start的道路上 。
                                        end = new RoadPoint(edges.get(0), Math.min(1d,
                                                1 - end.fraction() + (5d / edges.get(0).length())));
                                        edges.remove(1);
                                    } else { //否则将将start压缩到end的道路上
                                        start = new RoadPoint(edges.get(1), Math.max(0d, 1
                                                - start.fraction() - (5d / edges.get(1).length())));
                                        edges.remove(0);
                                    }
                                }
                                route = new Route(start, end, edges);
                            }
                        }

                        double beta = lambda == 0
                                ? Math.max(1d, candidates.one().time() - predecessors.one().time())
                                        / 1000
                                : 1 / lambda;

                        double transition = (1 / beta)
                                * Math.exp((-1.0) * route.cost(new TimePriority()) / beta);

                        map.put(candidate, new Tuple<>(new MatcherTransition(route), transition));

                        logger.trace("{} -> {} {} {}", predecessor.id(), candidate.id(),
                                route.length(), transition);
                        count.incrementAndGet();
                    }

                    transitions.put(predecessor, map);
                }
            });
        }
        if (!scheduler.sync()) {
            throw new RuntimeException();
        }

        sw.stop();

        logger.trace("{} transitions ({} ms)", count.get(), sw.ms());

        return transitions;
    }

    /**
     * Matches a full sequence of samples, {@link MatcherSample} objects and returns state
     * representation of the full matching which is a {@link KState} object.
     *
     * @param samples Sequence of samples, {@link MatcherSample} objects.
     * @param minDistance Minimum distance in meters between subsequent samples as criterion to
     *        match a sample. (Avoids unnecessary matching where samples are more dense than
     *        necessary.)
     * @param minInterval Minimum time interval in milliseconds between subsequent samples as
     *        criterion to match a sample. (Avoids unnecessary matching where samples are more dense
     *        than necessary.)
     * @return State representation of the full matching which is a {@link KState} object.
     */
    public MatcherKState mmatch(List<MatcherSample> samples, double minDistance, int minInterval) {
        Collections.sort(samples, new Comparator<MatcherSample>() {
            @Override
            public int compare(MatcherSample left, MatcherSample right) {
                return (int) (left.time() - right.time());
            }
        });

        MatcherKState state = new MatcherKState();

        for (MatcherSample sample : samples) {
            if (state.sample() != null && (spatial.distance(sample.point(),
                    state.sample().point()) < Math.max(0, minDistance)
                    || (sample.time() - state.sample().time()) < Math.max(0, minInterval))) {
                continue;
            }
            Set<MatcherCandidate> vector = execute(state.vector(), state.sample(), sample);
            state.update(vector, sample);
        }

        return state;
    }

    /**
     * todo 返回轨迹序列对应完整匹配序列，包括没有匹配上的null
     */
    public Tuple<List<MatcherSample>, List<MatcherCandidate>> getMatchSequence(List<MatcherSample> samples) {
        MatcherKState state = mmatch(samples, 0, 0);
        List<MatcherCandidate> sequence = state.sequence();

        return new Tuple<>(samples, sequence);
    }

    /**
     * todo 返回匹配上了的匹配点结果序列
     */
    public List<MatcherCandidate> getMatchCandidates(List<MatcherSample> samples) {
        MatcherKState state = mmatch(samples, 0, 0);
        return state.sequence();
    }
}
