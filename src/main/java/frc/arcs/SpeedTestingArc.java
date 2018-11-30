package frc.arcs;

import com.team319.follower.SrxMotionProfile;
import com.team319.follower.SrxTrajectory;

public class SpeedTestingArc extends SrxTrajectory {
	
	// WAYPOINTS:
	// (X,Y,degrees)
	// (2.00,13.50,0.00)
	// (5.00,16.50,89.99)
	// (2.00,19.50,179.98)
	
    public SpeedTestingArc() {
		super();
		this.highGear = true;
		centerProfile = new SrxMotionProfile(centerPoints.length, centerPoints);
	}

	
    public SpeedTestingArc(boolean flipped) {
		super();
		this.highGear = true;
		this.flipped = flipped;
		centerProfile = new SrxMotionProfile(centerPoints.length, centerPoints);
	}

	public boolean highGear = true;

	double[][] centerPoints = {
				{0.000,0.000,10.000,0.000},
				{0.293,2.934,10.000,0.000},
				{0.880,5.867,10.000,0.000},
				{1.760,8.801,10.000,0.000},
				{2.934,11.734,10.000,0.000},
				{4.400,14.668,10.000,0.000},
				{6.160,17.601,10.000,0.000},
				{8.214,20.535,10.000,0.000},
				{10.561,23.468,10.000,0.000},
				{13.201,26.402,10.000,0.010},
				{16.134,29.335,10.000,0.010},
				{19.361,32.269,10.000,0.010},
				{22.882,35.203,10.000,0.020},
				{26.695,38.136,10.000,0.030},
				{30.802,41.070,10.000,0.030},
				{35.203,44.003,10.000,0.040},
				{39.896,46.937,10.000,0.060},
				{44.883,49.870,10.000,0.070},
				{50.164,52.804,10.000,0.090},
				{55.737,55.737,10.000,0.110},
				{61.604,58.671,10.000,0.130},
				{67.765,61.604,10.000,0.160},
				{74.219,64.538,10.000,0.190},
				{80.966,67.472,10.000,0.230},
				{88.006,70.405,10.000,0.270},
				{95.340,73.339,10.000,0.320},
				{102.967,76.272,10.000,0.370},
				{110.888,79.206,10.000,0.430},
				{119.102,82.139,10.000,0.490},
				{127.609,85.073,10.000,0.570},
				{136.410,88.006,10.000,0.650},
				{145.504,90.940,10.000,0.730},
				{154.891,93.873,10.000,0.830},
				{164.572,96.807,10.000,0.940},
				{174.546,99.740,10.000,1.060},
				{184.813,102.674,10.000,1.180},
				{195.374,105.608,10.000,1.320},
				{206.228,108.541,10.000,1.480},
				{217.229,110.008,10.000,1.640},
				{228.230,110.008,10.000,1.810},
				{239.231,110.008,10.000,1.990},
				{250.231,110.008,10.000,2.180},
				{261.232,110.008,10.000,2.390},
				{272.233,110.008,10.000,2.600},
				{283.234,110.008,10.000,2.820},
				{294.234,110.008,10.000,3.050},
				{305.235,110.008,10.000,3.290},
				{316.236,110.008,10.000,3.540},
				{327.237,110.008,10.000,3.800},
				{338.238,110.008,10.000,4.070},
				{349.238,110.008,10.000,4.350},
				{360.239,110.008,10.000,4.650},
				{371.240,110.008,10.000,4.960},
				{382.241,110.008,10.000,5.270},
				{393.242,110.008,10.000,5.610},
				{404.242,110.008,10.000,5.950},
				{415.243,110.008,10.000,6.310},
				{426.244,110.008,10.000,6.680},
				{437.245,110.008,10.000,7.060},
				{448.246,110.008,10.000,7.460},
				{459.246,110.008,10.000,7.870},
				{470.247,110.008,10.000,8.300},
				{481.248,110.008,10.000,8.740},
				{492.249,110.008,10.000,9.200},
				{503.249,110.008,10.000,9.670},
				{514.250,110.008,10.000,10.160},
				{525.251,110.008,10.000,10.670},
				{536.252,110.008,10.000,11.190},
				{547.253,110.008,10.000,11.740},
				{558.253,110.008,10.000,12.300},
				{569.254,110.008,10.000,12.880},
				{580.255,110.008,10.000,13.480},
				{591.256,110.008,10.000,14.100},
				{602.257,110.008,10.000,14.740},
				{613.257,110.008,10.000,15.400},
				{624.258,110.008,10.000,16.080},
				{635.259,110.008,10.000,16.780},
				{646.260,110.008,10.000,17.510},
				{657.261,110.008,10.000,18.260},
				{668.261,110.008,10.000,19.030},
				{679.262,110.008,10.000,19.820},
				{690.263,110.008,10.000,20.640},
				{701.264,110.008,10.000,21.480},
				{712.264,110.008,10.000,22.340},
				{723.265,110.008,10.000,23.230},
				{734.266,110.008,10.000,24.150},
				{745.267,110.008,10.000,25.080},
				{756.268,110.008,10.000,26.040},
				{767.268,110.008,10.000,27.030},
				{778.269,110.008,10.000,28.040},
				{789.270,110.008,10.000,29.060},
				{800.271,110.008,10.000,30.120},
				{811.272,110.008,10.000,31.190},
				{822.272,110.008,10.000,32.280},
				{833.273,110.008,10.000,33.390},
				{844.274,110.008,10.000,34.520},
				{855.275,110.008,10.000,35.670},
				{866.276,110.008,10.000,36.830},
				{877.276,110.008,10.000,38.000},
				{888.277,110.008,10.000,39.190},
				{899.278,110.008,10.000,40.380},
				{910.279,110.008,10.000,41.590},
				{921.279,110.008,10.000,42.800},
				{932.280,110.008,10.000,44.010},
				{943.281,110.008,10.000,45.220},
				{954.282,110.008,10.000,46.440},
				{965.283,110.008,10.000,47.650},
				{976.283,110.008,10.000,48.860},
				{987.284,110.008,10.000,50.060},
				{998.285,110.008,10.000,51.250},
				{1009.286,110.008,10.000,52.430},
				{1020.287,110.008,10.000,53.600},
				{1031.287,110.008,10.000,54.760},
				{1042.288,110.008,10.000,55.900},
				{1053.289,110.008,10.000,57.020},
				{1064.290,110.008,10.000,58.120},
				{1075.291,110.008,10.000,59.210},
				{1086.291,110.008,10.000,60.270},
				{1097.292,110.008,10.000,61.320},
				{1108.293,110.008,10.000,62.340},
				{1119.294,110.008,10.000,63.340},
				{1130.294,110.008,10.000,64.310},
				{1141.295,110.008,10.000,65.260},
				{1152.296,110.008,10.000,66.190},
				{1163.297,110.008,10.000,67.090},
				{1174.298,110.008,10.000,67.970},
				{1185.298,110.008,10.000,68.830},
				{1196.299,110.008,10.000,69.660},
				{1207.300,110.008,10.000,70.470},
				{1218.301,110.008,10.000,71.260},
				{1229.302,110.008,10.000,72.020},
				{1240.302,110.008,10.000,72.760},
				{1251.303,110.008,10.000,73.470},
				{1262.304,110.008,10.000,74.170},
				{1273.305,110.008,10.000,74.840},
				{1284.306,110.008,10.000,75.500},
				{1295.306,110.008,10.000,76.130},
				{1306.307,110.008,10.000,76.740},
				{1317.308,110.008,10.000,77.330},
				{1328.309,110.008,10.000,77.900},
				{1339.309,110.008,10.000,78.460},
				{1350.310,110.008,10.000,79.000},
				{1361.311,110.008,10.000,79.510},
				{1372.312,110.008,10.000,80.010},
				{1383.313,110.008,10.000,80.500},
				{1394.313,110.008,10.000,80.970},
				{1405.314,110.008,10.000,81.420},
				{1416.315,110.008,10.000,81.850},
				{1427.316,110.008,10.000,82.280},
				{1438.317,110.008,10.000,82.680},
				{1449.317,110.008,10.000,83.070},
				{1460.318,110.008,10.000,83.450},
				{1471.319,110.008,10.000,83.820},
				{1482.320,110.008,10.000,84.170},
				{1493.321,110.008,10.000,84.510},
				{1504.321,110.008,10.000,84.840},
				{1515.322,110.008,10.000,85.150},
				{1526.323,110.008,10.000,85.450},
				{1537.324,110.008,10.000,85.740},
				{1548.324,110.008,10.000,86.020},
				{1559.325,110.008,10.000,86.290},
				{1570.326,110.008,10.000,86.550},
				{1581.327,110.008,10.000,86.790},
				{1592.328,110.008,10.000,87.030},
				{1603.328,110.008,10.000,87.260},
				{1614.329,110.008,10.000,87.470},
				{1625.330,110.008,10.000,87.680},
				{1636.331,110.008,10.000,87.880},
				{1647.332,110.008,10.000,88.060},
				{1658.332,110.008,10.000,88.240},
				{1669.333,110.008,10.000,88.410},
				{1680.334,110.008,10.000,88.570},
				{1691.335,110.008,10.000,88.720},
				{1702.336,110.008,10.000,88.860},
				{1713.336,110.008,10.000,89.000},
				{1724.044,107.074,10.000,89.120},
				{1734.458,104.141,10.000,89.230},
				{1744.579,101.207,10.000,89.330},
				{1754.406,98.274,10.000,89.420},
				{1763.940,95.340,10.000,89.500},
				{1773.181,92.407,10.000,89.570},
				{1782.128,89.473,10.000,89.640},
				{1790.782,86.540,10.000,89.700},
				{1799.142,83.606,10.000,89.750},
				{1807.210,80.672,10.000,89.790},
				{1814.984,77.739,10.000,89.830},
				{1822.464,74.805,10.000,89.860},
				{1829.651,71.872,10.000,89.890},
				{1836.545,68.938,10.000,89.920},
				{1843.146,66.005,10.000,89.940},
				{1849.453,63.071,10.000,89.950},
				{1855.467,60.138,10.000,89.960},
				{1861.187,57.204,10.000,89.970},
				{1866.614,54.271,10.000,89.980},
				{1871.748,51.337,10.000,89.990},
				{1876.588,48.403,10.000,89.990},
				{1881.135,45.470,10.000,89.990},
				{1885.389,42.536,10.000,89.990},
				{1885.389,42.536,10.000,89.990},
				{1889.056,36.669,10.000,89.990},
				{1892.723,36.669,10.000,89.990},
				{1896.389,36.669,10.000,90.000},
				{1900.056,36.669,10.000,90.000},
				{1903.723,36.669,10.000,90.010},
				{1907.390,36.669,10.000,90.010},
				{1911.057,36.669,10.000,90.020},
				{1914.724,36.669,10.000,90.030},
				{1918.391,36.669,10.000,90.040},
				{1922.058,36.669,10.000,90.040},
				{1925.725,36.669,10.000,90.060},
				{1929.392,36.669,10.000,90.070},
				{1933.059,36.669,10.000,90.080},
				{1936.726,36.669,10.000,90.090},
				{1940.393,36.669,10.000,90.110},
				{1944.060,36.669,10.000,90.120},
				{1947.726,36.669,10.000,90.140},
				{1951.393,36.669,10.000,90.160},
				{1955.060,36.669,10.000,90.170},
				{1958.727,36.669,10.000,90.190},
				{1962.394,36.669,10.000,90.210},
				{1966.061,36.669,10.000,90.230},
				{1969.728,36.669,10.000,90.250},
				{1973.395,36.669,10.000,90.280},
				{1977.062,36.669,10.000,90.300},
				{1980.729,36.669,10.000,90.330},
				{1984.396,36.669,10.000,90.350},
				{1988.063,36.669,10.000,90.380},
				{1991.730,36.669,10.000,90.400},
				{1995.397,36.669,10.000,90.430},
				{1999.063,36.669,10.000,90.460},
				{2002.730,36.669,10.000,90.490},
				{2006.397,36.669,10.000,90.520},
				{2010.064,36.669,10.000,90.550},
				{2013.731,36.669,10.000,90.590},
				{2017.398,36.669,10.000,90.620},
				{2021.065,36.669,10.000,90.660},
				{2024.732,36.669,10.000,90.690},
				{2028.399,36.669,10.000,90.730},
				{2032.066,36.669,10.000,90.770},
				{2035.733,36.669,10.000,90.800},
				{2039.400,36.669,10.000,90.840},
				{2043.067,36.669,10.000,90.880},
				{2046.734,36.669,10.000,90.920},
				{2050.401,36.669,10.000,90.970},
				{2054.067,36.669,10.000,91.010},
				{2057.734,36.669,10.000,91.050},
				{2061.401,36.669,10.000,91.100},
				{2065.068,36.669,10.000,91.150},
				{2068.735,36.669,10.000,91.190},
				{2072.402,36.669,10.000,91.240},
				{2076.069,36.669,10.000,91.290},
				{2079.736,36.669,10.000,91.340},
				{2083.403,36.669,10.000,91.390},
				{2087.070,36.669,10.000,91.440},
				{2090.737,36.669,10.000,91.500},
				{2094.404,36.669,10.000,91.550},
				{2098.071,36.669,10.000,91.600},
				{2101.738,36.669,10.000,91.660},
				{2105.404,36.669,10.000,91.720},
				{2109.071,36.669,10.000,91.780},
				{2112.738,36.669,10.000,91.830},
				{2116.405,36.669,10.000,91.890},
				{2120.072,36.669,10.000,91.960},
				{2123.739,36.669,10.000,92.020},
				{2127.406,36.669,10.000,92.080},
				{2131.073,36.669,10.000,92.150},
				{2134.740,36.669,10.000,92.210},
				{2138.407,36.669,10.000,92.280},
				{2142.074,36.669,10.000,92.340},
				{2145.741,36.669,10.000,92.410},
				{2149.408,36.669,10.000,92.480},
				{2153.075,36.669,10.000,92.550},
				{2156.741,36.669,10.000,92.620},
				{2160.408,36.669,10.000,92.700},
				{2164.075,36.669,10.000,92.770},
				{2167.742,36.669,10.000,92.850},
				{2171.409,36.669,10.000,92.920},
				{2175.076,36.669,10.000,93.000},
				{2178.743,36.669,10.000,93.080},
				{2182.410,36.669,10.000,93.160},
				{2186.077,36.669,10.000,93.240},
				{2189.744,36.669,10.000,93.320},
				{2193.411,36.669,10.000,93.400},
				{2197.078,36.669,10.000,93.490},
				{2200.745,36.669,10.000,93.570},
				{2204.412,36.669,10.000,93.660},
				{2208.079,36.669,10.000,93.750},
				{2211.745,36.669,10.000,93.840},
				{2215.412,36.669,10.000,93.930},
				{2219.079,36.669,10.000,94.020},
				{2222.746,36.669,10.000,94.110},
				{2226.413,36.669,10.000,94.210},
				{2230.080,36.669,10.000,94.300},
				{2233.747,36.669,10.000,94.400},
				{2237.414,36.669,10.000,94.500},
				{2241.081,36.669,10.000,94.590},
				{2244.748,36.669,10.000,94.690},
				{2248.415,36.669,10.000,94.800},
				{2252.082,36.669,10.000,94.900},
				{2255.749,36.669,10.000,95.000},
				{2259.416,36.669,10.000,95.110},
				{2263.082,36.669,10.000,95.220},
				{2266.749,36.669,10.000,95.320},
				{2270.416,36.669,10.000,95.430},
				{2274.083,36.669,10.000,95.550},
				{2277.750,36.669,10.000,95.660},
				{2281.417,36.669,10.000,95.770},
				{2285.084,36.669,10.000,95.890},
				{2288.751,36.669,10.000,96.000},
				{2292.418,36.669,10.000,96.120},
				{2296.085,36.669,10.000,96.240},
				{2299.752,36.669,10.000,96.360},
				{2303.419,36.669,10.000,96.490},
				{2307.086,36.669,10.000,96.610},
				{2310.753,36.669,10.000,96.740},
				{2314.419,36.669,10.000,96.860},
				{2318.086,36.669,10.000,96.990},
				{2321.753,36.669,10.000,97.120},
				{2325.420,36.669,10.000,97.250},
				{2329.087,36.669,10.000,97.390},
				{2332.754,36.669,10.000,97.520},
				{2336.421,36.669,10.000,97.660},
				{2340.088,36.669,10.000,97.800},
				{2343.755,36.669,10.000,97.940},
				{2347.422,36.669,10.000,98.080},
				{2351.089,36.669,10.000,98.220},
				{2354.756,36.669,10.000,98.370},
				{2358.423,36.669,10.000,98.510},
				{2362.090,36.669,10.000,98.660},
				{2365.756,36.669,10.000,98.810},
				{2369.423,36.669,10.000,98.970},
				{2373.090,36.669,10.000,99.120},
				{2376.757,36.669,10.000,99.270},
				{2380.424,36.669,10.000,99.430},
				{2384.091,36.669,10.000,99.590},
				{2387.758,36.669,10.000,99.750},
				{2391.425,36.669,10.000,99.910},
				{2395.092,36.669,10.000,100.080},
				{2398.759,36.669,10.000,100.250},
				{2402.426,36.669,10.000,100.410},
				{2406.093,36.669,10.000,100.580},
				{2409.760,36.669,10.000,100.760},
				{2413.427,36.669,10.000,100.930},
				{2417.094,36.669,10.000,101.110},
				{2420.760,36.669,10.000,101.280},
				{2424.427,36.669,10.000,101.460},
				{2428.094,36.669,10.000,101.650},
				{2431.761,36.669,10.000,101.830},
				{2435.428,36.669,10.000,102.020},
				{2439.095,36.669,10.000,102.210},
				{2442.762,36.669,10.000,102.400},
				{2446.429,36.669,10.000,102.590},
				{2450.096,36.669,10.000,102.780},
				{2453.763,36.669,10.000,102.980},
				{2457.430,36.669,10.000,103.180},
				{2461.097,36.669,10.000,103.380},
				{2464.764,36.669,10.000,103.580},
				{2468.431,36.669,10.000,103.790},
				{2472.097,36.669,10.000,104.000},
				{2475.764,36.669,10.000,104.210},
				{2479.431,36.669,10.000,104.420},
				{2483.098,36.669,10.000,104.630},
				{2486.765,36.669,10.000,104.850},
				{2490.432,36.669,10.000,105.070},
				{2494.099,36.669,10.000,105.290},
				{2497.766,36.669,10.000,105.510},
				{2501.433,36.669,10.000,105.740},
				{2505.100,36.669,10.000,105.970},
				{2508.767,36.669,10.000,106.200},
				{2512.434,36.669,10.000,106.430},
				{2516.101,36.669,10.000,106.670},
				{2519.768,36.669,10.000,106.910},
				{2523.434,36.669,10.000,107.150},
				{2527.101,36.669,10.000,107.390},
				{2530.768,36.669,10.000,107.640},
				{2534.435,36.669,10.000,107.890},
				{2538.102,36.669,10.000,108.140},
				{2541.769,36.669,10.000,108.390},
				{2545.436,36.669,10.000,108.650},
				{2549.103,36.669,10.000,108.900},
				{2552.770,36.669,10.000,109.170},
				{2556.437,36.669,10.000,109.430},
				{2560.104,36.669,10.000,109.700},
				{2563.771,36.669,10.000,109.960},
				{2567.438,36.669,10.000,110.240},
				{2571.105,36.669,10.000,110.510},
				{2574.771,36.669,10.000,110.790},
				{2578.438,36.669,10.000,111.070},
				{2582.105,36.669,10.000,111.350},
				{2585.772,36.669,10.000,111.630},
				{2589.439,36.669,10.000,111.920},
				{2593.106,36.669,10.000,112.210},
				{2596.773,36.669,10.000,112.500},
				{2600.440,36.669,10.000,112.800},
				{2604.107,36.669,10.000,113.100},
				{2607.774,36.669,10.000,113.400},
				{2611.441,36.669,10.000,113.700},
				{2615.108,36.669,10.000,114.010},
				{2618.775,36.669,10.000,114.310},
				{2622.442,36.669,10.000,114.620},
				{2626.109,36.669,10.000,114.940},
				{2629.775,36.669,10.000,115.260},
				{2633.442,36.669,10.000,115.570},
				{2637.109,36.669,10.000,115.900},
				{2640.776,36.669,10.000,116.220},
				{2644.443,36.669,10.000,116.550},
				{2648.110,36.669,10.000,116.880},
				{2651.777,36.669,10.000,117.210},
				{2655.444,36.669,10.000,117.540},
				{2659.111,36.669,10.000,117.880},
				{2662.778,36.669,10.000,118.220},
				{2666.445,36.669,10.000,118.560},
				{2670.112,36.669,10.000,118.910},
				{2673.779,36.669,10.000,119.250},
				{2677.446,36.669,10.000,119.600},
				{2681.112,36.669,10.000,119.950},
				{2684.779,36.669,10.000,120.310},
				{2688.446,36.669,10.000,120.670},
				{2692.113,36.669,10.000,121.020},
				{2695.780,36.669,10.000,121.390},
				{2699.447,36.669,10.000,121.750},
				{2703.114,36.669,10.000,122.110},
				{2706.781,36.669,10.000,122.480},
				{2710.448,36.669,10.000,122.850},
				{2714.115,36.669,10.000,123.220},
				{2717.782,36.669,10.000,123.600},
				{2721.449,36.669,10.000,123.970},
				{2725.116,36.669,10.000,124.350},
				{2728.783,36.669,10.000,124.730},
				{2732.449,36.669,10.000,125.110},
				{2736.116,36.669,10.000,125.490},
				{2739.783,36.669,10.000,125.880},
				{2743.450,36.669,10.000,126.260},
				{2747.117,36.669,10.000,126.650},
				{2750.784,36.669,10.000,127.040},
				{2754.451,36.669,10.000,127.430},
				{2758.118,36.669,10.000,127.820},
				{2761.785,36.669,10.000,128.220},
				{2765.452,36.669,10.000,128.610},
				{2769.119,36.669,10.000,129.010},
				{2772.786,36.669,10.000,129.410},
				{2776.453,36.669,10.000,129.800},
				{2780.120,36.669,10.000,130.200},
				{2783.786,36.669,10.000,130.600},
				{2787.453,36.669,10.000,131.000},
				{2791.120,36.669,10.000,131.410},
				{2794.787,36.669,10.000,131.810},
				{2798.454,36.669,10.000,132.210},
				{2802.121,36.669,10.000,132.610},
				{2805.788,36.669,10.000,133.020},
				{2809.455,36.669,10.000,133.420},
				{2813.122,36.669,10.000,133.830},
				{2816.789,36.669,10.000,134.230},
				{2820.456,36.669,10.000,134.640},
				{2824.123,36.669,10.000,135.040},
				{2827.790,36.669,10.000,135.450},
				{2831.457,36.669,10.000,135.850},
				{2835.124,36.669,10.000,136.260},
				{2838.790,36.669,10.000,136.660},
				{2842.457,36.669,10.000,137.070},
				{2846.124,36.669,10.000,137.470},
				{2849.791,36.669,10.000,137.870},
				{2853.458,36.669,10.000,138.280},
				{2857.125,36.669,10.000,138.680},
				{2860.792,36.669,10.000,139.080},
				{2864.459,36.669,10.000,139.480},
				{2868.126,36.669,10.000,139.880},
				{2871.793,36.669,10.000,140.280},
				{2875.460,36.669,10.000,140.680},
				{2879.127,36.669,10.000,141.070},
				{2882.794,36.669,10.000,141.470},
				{2886.461,36.669,10.000,141.860},
				{2890.127,36.669,10.000,142.260},
				{2893.794,36.669,10.000,142.650},
				{2897.461,36.669,10.000,143.040},
				{2901.128,36.669,10.000,143.430},
				{2904.795,36.669,10.000,143.810},
				{2908.462,36.669,10.000,144.200},
				{2912.129,36.669,10.000,144.580},
				{2915.796,36.669,10.000,144.970},
				{2919.463,36.669,10.000,145.350},
				{2923.130,36.669,10.000,145.730},
				{2926.797,36.669,10.000,146.100},
				{2930.464,36.669,10.000,146.480},
				{2934.131,36.669,10.000,146.850},
				{2937.798,36.669,10.000,147.220},
				{2941.464,36.669,10.000,147.590},
				{2945.131,36.669,10.000,147.960},
				{2948.798,36.669,10.000,148.320},
				{2952.465,36.669,10.000,148.690},
				{2956.132,36.669,10.000,149.050},
				{2959.799,36.669,10.000,149.400},
				{2963.466,36.669,10.000,149.760},
				{2967.133,36.669,10.000,150.110},
				{2970.800,36.669,10.000,150.460},
				{2974.467,36.669,10.000,150.810},
				{2978.134,36.669,10.000,151.160},
				{2981.801,36.669,10.000,151.500},
				{2985.468,36.669,10.000,151.840},
				{2989.135,36.669,10.000,152.180},
				{2992.801,36.669,10.000,152.520},
				{2996.468,36.669,10.000,152.850},
				{3000.135,36.669,10.000,153.190},
				{3003.802,36.669,10.000,153.510},
				{3007.469,36.669,10.000,153.840},
				{3011.136,36.669,10.000,154.160},
				{3014.803,36.669,10.000,154.480},
				{3018.470,36.669,10.000,154.800},
				{3022.137,36.669,10.000,155.120},
				{3025.804,36.669,10.000,155.430},
				{3029.471,36.669,10.000,155.740},
				{3033.138,36.669,10.000,156.050},
				{3036.805,36.669,10.000,156.350},
				{3040.472,36.669,10.000,156.660},
				{3044.139,36.669,10.000,156.960},
				{3047.805,36.669,10.000,157.250},
				{3051.472,36.669,10.000,157.550},
				{3055.139,36.669,10.000,157.840},
				{3058.806,36.669,10.000,158.130},
				{3062.473,36.669,10.000,158.420},
				{3066.140,36.669,10.000,158.700},
				{3069.807,36.669,10.000,158.980},
				{3073.474,36.669,10.000,159.260},
				{3077.141,36.669,10.000,159.530},
				{3080.808,36.669,10.000,159.810},
				{3084.475,36.669,10.000,160.080},
				{3088.142,36.669,10.000,160.350},
				{3091.809,36.669,10.000,160.610},
				{3095.476,36.669,10.000,160.870},
				{3099.142,36.669,10.000,161.140},
				{3102.809,36.669,10.000,161.390},
				{3106.476,36.669,10.000,161.650},
				{3110.143,36.669,10.000,161.900},
				{3113.810,36.669,10.000,162.150},
				{3117.477,36.669,10.000,162.400},
				{3121.144,36.669,10.000,162.640},
				{3124.811,36.669,10.000,162.890},
				{3128.478,36.669,10.000,163.130},
				{3132.145,36.669,10.000,163.360},
				{3135.812,36.669,10.000,163.600},
				{3139.479,36.669,10.000,163.830},
				{3143.146,36.669,10.000,164.060},
				{3146.813,36.669,10.000,164.290},
				{3150.479,36.669,10.000,164.510},
				{3154.146,36.669,10.000,164.740},
				{3157.813,36.669,10.000,164.960},
				{3161.480,36.669,10.000,165.180},
				{3165.147,36.669,10.000,165.390},
				{3168.814,36.669,10.000,165.610},
				{3172.481,36.669,10.000,165.820},
				{3176.148,36.669,10.000,166.030},
				{3179.815,36.669,10.000,166.240},
				{3183.482,36.669,10.000,166.440},
				{3187.149,36.669,10.000,166.640},
				{3190.816,36.669,10.000,166.840},
				{3194.483,36.669,10.000,167.040},
				{3198.150,36.669,10.000,167.240},
				{3201.817,36.669,10.000,167.430},
				{3205.483,36.669,10.000,167.620},
				{3209.150,36.669,10.000,167.810},
				{3212.817,36.669,10.000,168.000},
				{3216.484,36.669,10.000,168.190},
				{3220.151,36.669,10.000,168.370},
				{3223.818,36.669,10.000,168.550},
				{3227.485,36.669,10.000,168.730},
				{3231.152,36.669,10.000,168.910},
				{3234.819,36.669,10.000,169.080},
				{3238.486,36.669,10.000,169.260},
				{3242.153,36.669,10.000,169.430},
				{3245.820,36.669,10.000,169.600},
				{3249.487,36.669,10.000,169.770},
				{3253.154,36.669,10.000,169.930},
				{3256.820,36.669,10.000,170.100},
				{3260.487,36.669,10.000,170.260},
				{3264.154,36.669,10.000,170.420},
				{3267.821,36.669,10.000,170.580},
				{3271.488,36.669,10.000,170.730},
				{3275.155,36.669,10.000,170.890},
				{3278.822,36.669,10.000,171.040},
				{3282.489,36.669,10.000,171.190},
				{3286.156,36.669,10.000,171.340},
				{3289.823,36.669,10.000,171.490},
				{3293.490,36.669,10.000,171.640},
				{3297.157,36.669,10.000,171.780},
				{3300.824,36.669,10.000,171.920},
				{3304.491,36.669,10.000,172.070},
				{3308.157,36.669,10.000,172.210},
				{3311.824,36.669,10.000,172.340},
				{3315.491,36.669,10.000,172.480},
				{3319.158,36.669,10.000,172.610},
				{3322.825,36.669,10.000,172.750},
				{3326.492,36.669,10.000,172.880},
				{3330.159,36.669,10.000,173.010},
				{3333.826,36.669,10.000,173.140},
				{3337.493,36.669,10.000,173.260},
				{3341.160,36.669,10.000,173.390},
				{3344.827,36.669,10.000,173.510},
				{3348.494,36.669,10.000,173.630},
				{3352.161,36.669,10.000,173.760},
				{3355.828,36.669,10.000,173.870},
				{3359.494,36.669,10.000,173.990},
				{3363.161,36.669,10.000,174.110},
				{3366.828,36.669,10.000,174.220},
				{3370.495,36.669,10.000,174.340},
				{3374.162,36.669,10.000,174.450},
				{3377.829,36.669,10.000,174.560},
				{3381.496,36.669,10.000,174.670},
				{3385.163,36.669,10.000,174.780},
				{3388.830,36.669,10.000,174.880},
				{3392.497,36.669,10.000,174.990},
				{3396.164,36.669,10.000,175.090},
				{3399.831,36.669,10.000,175.200},
				{3403.498,36.669,10.000,175.300},
				{3407.165,36.669,10.000,175.400},
				{3410.832,36.669,10.000,175.500},
				{3414.498,36.669,10.000,175.590},
				{3418.165,36.669,10.000,175.690},
				{3421.832,36.669,10.000,175.780},
				{3425.499,36.669,10.000,175.880},
				{3429.166,36.669,10.000,175.970},
				{3432.833,36.669,10.000,176.060},
				{3436.500,36.669,10.000,176.150},
				{3440.167,36.669,10.000,176.240},
				{3443.834,36.669,10.000,176.330},
				{3447.501,36.669,10.000,176.410},
				{3451.168,36.669,10.000,176.500},
				{3454.835,36.669,10.000,176.580},
				{3458.502,36.669,10.000,176.670},
				{3462.169,36.669,10.000,176.750},
				{3465.835,36.669,10.000,176.830},
				{3469.502,36.669,10.000,176.910},
				{3473.169,36.669,10.000,176.990},
				{3476.836,36.669,10.000,177.060},
				{3480.503,36.669,10.000,177.140},
				{3484.170,36.669,10.000,177.210},
				{3487.837,36.669,10.000,177.290},
				{3491.504,36.669,10.000,177.360},
				{3495.171,36.669,10.000,177.430},
				{3498.838,36.669,10.000,177.500},
				{3502.505,36.669,10.000,177.570},
				{3506.172,36.669,10.000,177.640},
				{3509.839,36.669,10.000,177.710},
				{3513.506,36.669,10.000,177.770},
				{3517.172,36.669,10.000,177.840},
				{3520.839,36.669,10.000,177.900},
				{3524.506,36.669,10.000,177.960},
				{3528.173,36.669,10.000,178.030},
				{3531.840,36.669,10.000,178.090},
				{3535.507,36.669,10.000,178.150},
				{3539.174,36.669,10.000,178.210},
				{3542.841,36.669,10.000,178.260},
				{3546.508,36.669,10.000,178.320},
				{3550.175,36.669,10.000,178.380},
				{3553.842,36.669,10.000,178.430},
				{3557.509,36.669,10.000,178.480},
				{3561.176,36.669,10.000,178.540},
				{3564.843,36.669,10.000,178.590},
				{3568.509,36.669,10.000,178.640},
				{3572.176,36.669,10.000,178.690},
				{3575.843,36.669,10.000,178.740},
				{3579.510,36.669,10.000,178.790},
				{3583.177,36.669,10.000,178.830},
				{3586.844,36.669,10.000,178.880},
				{3590.511,36.669,10.000,178.920},
				{3594.178,36.669,10.000,178.970},
				{3597.845,36.669,10.000,179.010},
				{3601.512,36.669,10.000,179.050},
				{3605.179,36.669,10.000,179.090},
				{3608.846,36.669,10.000,179.130},
				{3612.513,36.669,10.000,179.170},
				{3616.180,36.669,10.000,179.210},
				{3619.847,36.669,10.000,179.250},
				{3623.513,36.669,10.000,179.280},
				{3627.180,36.669,10.000,179.320},
				{3630.847,36.669,10.000,179.350},
				{3634.514,36.669,10.000,179.390},
				{3638.181,36.669,10.000,179.420},
				{3641.848,36.669,10.000,179.450},
				{3645.515,36.669,10.000,179.480},
				{3649.182,36.669,10.000,179.510},
				{3652.849,36.669,10.000,179.540},
				{3656.516,36.669,10.000,179.570},
				{3660.183,36.669,10.000,179.600},
				{3663.850,36.669,10.000,179.620},
				{3667.517,36.669,10.000,179.650},
				{3671.184,36.669,10.000,179.670},
				{3674.850,36.669,10.000,179.700},
				{3678.517,36.669,10.000,179.720},
				{3682.184,36.669,10.000,179.740},
				{3685.851,36.669,10.000,179.760},
				{3689.518,36.669,10.000,179.780},
				{3693.185,36.669,10.000,179.800},
				{3696.852,36.669,10.000,179.820},
				{3700.519,36.669,10.000,179.830},
				{3704.186,36.669,10.000,179.850},
				{3707.853,36.669,10.000,179.860},
				{3711.520,36.669,10.000,179.880},
				{3715.187,36.669,10.000,179.890},
				{3718.854,36.669,10.000,179.900},
				{3722.521,36.669,10.000,179.920},
				{3726.187,36.669,10.000,179.930},
				{3729.854,36.669,10.000,179.940},
				{3733.521,36.669,10.000,179.940},
				{3737.188,36.669,10.000,179.950},
				{3740.855,36.669,10.000,179.960},
				{3744.522,36.669,10.000,179.960},
				{3747.896,33.736,10.000,179.970},
				{3750.976,30.802,10.000,179.970},
				{3753.763,27.869,10.000,179.980},
				{3756.256,24.935,10.000,179.980},
				{3758.456,22.002,10.000,179.980},
				{3760.363,19.068,10.000,179.980},
				{3761.977,16.134,10.000,179.980},
				{3763.297,13.201,10.000,179.980},
				{3764.324,10.267,10.000,179.980},
				{3765.057,7.334,10.000,179.980},
				{3765.497,4.400,10.000,179.980}		};

}