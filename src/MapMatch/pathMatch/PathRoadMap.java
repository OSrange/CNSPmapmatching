package MapMatch.pathMatch;

import com.bmwcarit.barefoot.roadmap.Road;
import com.bmwcarit.barefoot.roadmap.RoadMap;
import com.bmwcarit.barefoot.roadmap.RoadPoint;
import com.bmwcarit.barefoot.spatial.QuadTreeIndex;
import com.bmwcarit.barefoot.spatial.SpatialIndex;
import com.bmwcarit.barefoot.util.Tuple;
import com.esri.core.geometry.Point;

import java.io.Serializable;
import java.util.*;

/**
 * 构造只有一条路径的地图
 */
public class PathRoadMap extends RoadMap implements Serializable {

    final double radius; 
    private Index index = null; 

    public PathRoadMap(List<Road> path, double radius) { 
        this.radius = radius;
        load(path);
    }

    private void load(List<Road> path) {
        for (Road road : path) {
            Road edge = new Road(road.base(), road.heading());
            add(edge);
        }
    }

    @Override
    public PathRoadMap construct() {
        superConstruct();

        
        index = new Index();
        for (Road road : edges.values()) {
            index.put(road);
        }
        return this;
    }

    /**
     * Graph中的construct方法
     */
    private PathRoadMap superConstruct() {
        
        Map<Long, ArrayList<Road>> map = new HashMap<>();

        for (Road edge : edges.values()) {
            if (!map.containsKey(edge.source())) {
                map.put(edge.source(), new ArrayList<>(Arrays.asList(edge)));
            } else {
                map.get(edge.source()).add(edge);
            }
        }

        for (ArrayList<Road> edges : map.values()) {
            for (int i = 1; i < edges.size(); ++i) {
                
                edges.get(i - 1).neighbor(edges.get(i));
                ArrayList<Road> successors = map.get(edges.get(i - 1).target());
                edges.get(i - 1).successor(successors != null ? successors.get(0) : null);
            }

            
            
            edges.get(edges.size() - 1).neighbor(edges.get(0));
            ArrayList<Road> successors = map.get(edges.get(edges.size() - 1).target());
            edges.get(edges.size() - 1).successor(successors != null ? successors.get(0) : null);
        }

        return this;
    }

    @Override
    public SpatialIndex<RoadPoint> spatial() {
        if (index == null)
            throw new RuntimeException("index not constructed");
        else
            return index;
    }

    private class Index implements SpatialIndex<RoadPoint>, Serializable {
        private static final long serialVersionUID = 1L;
        private final QuadTreeIndex index = new QuadTreeIndex();

        public void put(Road road) {
            int id = (int) road.base().id();

            if (index.contains(id)) {
                return;
            }

            index.add(id, road.base().wkb());
        }

        public void clear() {
            index.clear();
        }

        private Set<RoadPoint> split(Set<Tuple<Integer, Double>> points) {
            Set<RoadPoint> neighbors = new HashSet<>();

            /*
             * This uses the road
             */
            for (Tuple<Integer, Double> point : points) {
                Road road = edges.get((long) point.one() * 2);
                if (road != null)
                    neighbors.add(new RoadPoint(road, point.two()));

                if (edges.containsKey((long) point.one() * 2 + 1)) {
                    neighbors.add(new RoadPoint(edges.get((long) point.one() * 2 + 1),
                            1.0 - point.two()));
                }
            }

            return neighbors;
        }

        @Override
        public Set<RoadPoint> nearest(Point c) {
            return split(index.nearest(c));
        }

        @Override
        public Set<RoadPoint> radius(Point c, double r) {
            return split(index.radius(c, r));
        }

        @Override
        public Set<RoadPoint> knearest(Point c, int k) {
            return split(index.knearest(c, k));
        }
    };
}
