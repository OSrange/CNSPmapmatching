package utils;
import com.esri.core.geometry.Point;

import java.io.*;
import java.text.SimpleDateFormat;
import java.util.*;

/**
 * 通用工具方法。非必要时不用Scala,能用Java就用Java，毕竟Scala能兼容Java，但是Java里调用不了Scala的方法
 */
public class CommonUtils {

    public static InputStream getResourceFilePath(String fileName) throws FileNotFoundException {
        return CommonUtils.class.getClassLoader().getResourceAsStream(fileName);
    }

    /**
     * 根据路径 不存在则创建文件
     * @param filePath
     * @return
     */
    public static File createFile(String filePath) {
        File file = new File(filePath);
        if (!file.exists()) {
            
            if (!file.getParentFile().exists()) {
                
                file.getParentFile().mkdirs();
            }
            try {
                
                file.createNewFile();
            } catch (IOException e) {
                e.printStackTrace();
            }
        }
        return file;
    }


    /**
     * 不存在则创建文件及其路径 存在则追加写
     */
    public static FileWriter getAppendWriter(String filePath) throws IOException {
        CommonUtils.createFile(filePath);
        return new FileWriter(filePath, true);
    }

    /**
     * 不存在则创建文件及其路径 存在则覆盖写
     */
    public static FileWriter getOverWriter(String filePath) throws IOException {
        CommonUtils.createFile(filePath);
        return new FileWriter(filePath, false);
    }

    /**
     *  java 获取 获取某年某月 所有日期（yyyy-mm-dd格式字符串）
     * @param year
     * @param month
     * @return
     */
    public static List<String> getMonthFullDay(int year , int month){
        SimpleDateFormat dateFormatYYYYMMDD = new SimpleDateFormat("yyyy-MM-dd");
        List<String> fullDayList = new ArrayList<String>(32);
        
        Calendar cal = Calendar.getInstance();
        cal.clear();
        cal.set(Calendar.YEAR, year);
        
        cal.set(Calendar.MONTH, month-1 );
        
        cal.set(Calendar.DAY_OF_MONTH,1);
        int count = cal.getActualMaximum(Calendar.DAY_OF_MONTH);
        for (int j = 1; j <= count ; j++) {
            fullDayList.add(dateFormatYYYYMMDD.format(cal.getTime()));
            cal.add(Calendar.DAY_OF_MONTH,1);
        }
        return fullDayList;
    }

    /**
     * 获取[startDate, endDate]区间内的所有日期（yyyy-mm-dd格式字符串）
     */
    public static List<String> getDays(String startDate, String endDate) {
        SimpleDateFormat sdf = new SimpleDateFormat("yyyy-MM-dd");
        List<String> dateList = new ArrayList<String>();
        try {
            Date dateOne = sdf.parse(startDate);
            Date dateTwo = sdf.parse(endDate);
            Calendar calendar = Calendar.getInstance();
            calendar.setTime(dateOne);
            dateList.add(startDate);
            while (dateTwo.after(calendar.getTime())) {
                calendar.add(Calendar.DAY_OF_MONTH, 1);
                dateList.add(sdf.format(calendar.getTime()));
            }
        } catch (Exception e) {
            e.printStackTrace();
        }
        return dateList;
    }

    /**
     * 将指定pattern的时间字符串time转换为时间戳
     */
    public static long timeToTS(String time, String pattern) throws Exception {
        SimpleDateFormat simpleDateFormat = new SimpleDateFormat(pattern);
        simpleDateFormat.setTimeZone(TimeZone.getTimeZone("Asia/Shanghai"));
        return simpleDateFormat.parse(time).getTime();
    }

    /**
     * 将时间戳转换为时间
     */
    public static String stampToTime(long timestamp, String pattern) throws Exception{
        SimpleDateFormat simpleDateFormat = new SimpleDateFormat(pattern);
        
        Date date = new Date(timestamp);
        return simpleDateFormat.format(date);
    }

    /**
     * 将时间戳转换为时间 默认为本地时间 即东八区时间 即北京/上海时间
     */
    public static String stampToTime(long timestamp) throws Exception{
        SimpleDateFormat simpleDateFormat = new SimpleDateFormat("yyyy-MM-dd HH:mm:ss");
        
        Date date = new Date(timestamp);
        
        return simpleDateFormat.format(date);
    }

    public static int getTimeSlot(long time) { 
        try {
            String str = stampToTime(time, "HH:mm");
            int hour = Integer.parseInt(str.split(":")[0]);
            int minute = Integer.parseInt(str.split(":")[1]);
            
            int minutesSinceDayStart = hour * 60 + minute;
            
            return minutesSinceDayStart / 5;
        } catch (Exception e) {
            e.printStackTrace();
        }
        return -1;
    }

    /**
     * Calculate distance between two points in latitude and longitude taking
     * into account height difference. If you are not interested in height
     * difference pass 0.0. Uses Haversine method as its base.
     *
     * lat1, lon1 Start point lat2, lon2 End point el1 Start altitude in meters
     * el2 End altitude in meters
     * @returns 输出单位：m
     * from : https:
     */
    public static double gpsDistance(double lon1, double lat1, double lon2, double lat2) {

        final double R = 6371.137; 

        double latDistance = Math.toRadians(lat2 - lat1);
        double lonDistance = Math.toRadians(lon2 - lon1);
        double a = Math.sin(latDistance / 2) * Math.sin(latDistance / 2)
                + Math.cos(Math.toRadians(lat1)) * Math.cos(Math.toRadians(lat2))
                * Math.sin(lonDistance / 2) * Math.sin(lonDistance / 2);
        double c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));
        double distance = R * c * 1000; 


        distance = Math.pow(distance, 2);

        return Math.sqrt(distance);
    }

    public static double toRadians(double degrees) {
        return degrees * (Math.PI / 180);
    }

    /**
     * 计算两个坐标点连线的方位角
     */
    public static double calculateAzimuth(Point p1, Point p2) {
        double longitude1 = toRadians(p1.getX());
        double longitude2 = toRadians(p2.getX());
        double latitude1 = toRadians(p1.getY());
        double latitude2 = toRadians(p2.getY());

        double longDiff = longitude2 - longitude1;
        double y = Math.sin(longDiff) * Math.cos(latitude2);
        double x = Math.cos(latitude1) * Math.sin(latitude2) - Math.sin(latitude1) * Math.cos(latitude2) * Math.cos(longDiff);

        return (Math.toDegrees(Math.atan2(y, x)) + 360) % 360;
    }

    /**
     * 计算方位角差异 范围[0, 180]
     */
    public static double azimuthDifference(double azimuth1, double azimuth2) {
        double difference = Math.abs(azimuth1 - azimuth2);
        if (difference > 180) {
            difference = 360 - difference;
        }
        return difference;
    }
}
