package test;

import MapMatch.shortpath.CNSPMatcher;
import MapMatch.shortpath.ClientMapMatcher;
import com.bmwcarit.barefoot.matcher.MatcherCandidate;
import com.bmwcarit.barefoot.matcher.MatcherSample;
import com.esri.core.geometry.Point;

import java.io.*;
import java.util.ArrayList;
import java.util.List;

/**
 * 测试用例
 */
public class SPMatchTest {

    static String mapPath = "testData/overpass.bfmap";

    public static String path = "testData/";
    static String fileName ="TrajectorySample";
    static String filePath = path + fileName + ".txt";

    public static void main(String[] args) throws IOException {
        List<MatcherSample> samples = new ArrayList<>();
        BufferedReader reader = new BufferedReader(new FileReader(filePath));
        String line;

        int id = 0;
        while ((line = reader.readLine()) != null) {
            String[] ss = line.split(",");

            

            samples.add(new MatcherSample(String.valueOf(id++), Long.parseLong(ss[1]), new Point(Double.parseDouble(ss[2]), Double.parseDouble(ss[3]))));
        }

        ClientMapMatcher<CNSPMatcher> ivMatcher = new ClientMapMatcher<>(mapPath, CNSPMatcher.class);

        List<MatcherCandidate> matchSequence = ivMatcher.getMatchCandidates(samples);

        BufferedWriter writer = new BufferedWriter(new FileWriter(path + fileName + "Result.txt"));
        writer.write("编号,时间戳,定位点经度,定位点纬度,匹配点经度,匹配点纬度,匹配道路osmid,道路方向,道路source,道路target");writer.newLine();
        for (int i = 0;i < matchSequence.size();i++) {
            MatcherCandidate c = matchSequence.get(i);
            MatcherSample s = c.sample();
            String[] formattedArray = new String[] {
                    s.id(),
                    Long.toString(s.time()),
                    String.format("%.6f", s.point().getX()),
                    String.format("%.6f", s.point().getY()),
                    String.format("%.6f", c.point().geometry().getX()),
                    String.format("%.6f", c.point().geometry().getY()),
                    String.valueOf(c.point().edge().base().refid()),
                    c.point().edge().heading().toString(),
                    String.valueOf(c.point().edge().source()),
                    String.valueOf(c.point().edge().target())
            };
            writer.write(String.join(",", formattedArray) + "\n");
        }
        writer.close();
    }
}
