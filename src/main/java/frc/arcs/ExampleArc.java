package frc.arcs;

import frc.models.SrxMotionProfile;
import frc.models.SrxTrajectory;

public class ExampleArc extends SrxTrajectory {
	
	// WAYPOINTS:
	// (X,Y,degrees)
	// (1.63,3.79,0.00)
	// (4.63,3.79,0.00)
	// (9.63,8.79,0.00)
	
    public ExampleArc() {
		super();
		this.highGear = true;
		centerProfile = new SrxMotionProfile(centerPoints.length, centerPoints);
	}

	
    public ExampleArc(boolean flipped) {
		super();
		this.highGear = true;
		this.flipped = flipped;
		centerProfile = new SrxMotionProfile(centerPoints.length, centerPoints);
	}

	public boolean highGear = true;

	double[][] centerPoints = {
				{0.000,0.000,10.000,0.000},
				{15.500,154.995,10.000,0.000},
				{46.499,309.991,10.000,0.000},
				{92.997,464.986,10.000,0.000},
				{154.995,619.982,10.000,0.000},
				{232.493,774.977,10.000,0.000},
				{325.490,929.973,10.000,0.000},
				{433.987,1084.968,10.000,0.000},
				{557.984,1239.963,10.000,0.000},
				{697.479,1394.959,10.000,0.000},
				{852.475,1549.954,10.000,0.000},
				{1022.970,1704.950,10.000,0.000},
				{1208.964,1859.945,10.000,0.000},
				{1410.458,2014.940,10.000,0.000},
				{1627.452,2169.936,10.000,0.000},
				{1859.945,2324.931,10.000,0.000},
				{2107.938,2479.927,10.000,0.000},
				{2371.430,2634.922,10.000,0.000},
				{2650.422,2789.918,10.000,0.000},
				{2944.913,2944.913,10.000,0.000},
				{3254.904,3099.908,10.000,0.000},
				{3580.394,3254.904,10.000,0.000},
				{3921.384,3409.899,10.000,0.000},
				{4277.874,3564.895,10.000,0.000},
				{4649.863,3719.890,10.000,0.000},
				{5037.351,3874.886,10.000,0.000},
				{5440.339,4029.881,10.000,0.000},
				{5858.827,4184.876,10.000,0.000},
				{6292.814,4339.872,10.000,0.000},
				{6742.301,4494.867,10.000,0.000},
				{7207.287,4649.863,10.000,0.000},
				{7687.773,4804.858,10.000,0.000},
				{8183.758,4959.854,10.000,0.000},
				{8695.243,5114.849,10.000,0.000},
				{9222.228,5269.844,10.000,0.000},
				{9764.712,5424.840,10.000,0.000},
				{10322.695,5579.835,10.000,0.000},
				{10896.178,5734.831,10.000,0.000},
				{11485.161,5889.826,10.000,0.000},
				{12089.643,6044.821,10.000,0.000},
				{12709.625,6199.817,10.000,0.000},
				{13345.106,6354.812,10.000,0.000},
				{13996.087,6509.808,10.000,0.000},
				{14662.567,6664.803,10.000,0.000},
				{15344.547,6819.799,10.000,0.000},
				{16042.026,6974.794,10.000,0.000},
				{16755.005,7129.789,10.000,0.000},
				{17483.484,7284.785,10.000,0.000},
				{18227.462,7439.780,10.000,0.000},
				{18986.939,7594.776,10.000,0.000},
				{19761.916,7749.771,10.000,0.000},
				{20552.393,7904.767,10.000,0.000},
				{21358.369,8059.762,10.000,0.000},
				{22179.845,8214.757,10.000,0.000},
				{23016.820,8369.753,10.000,0.000},
				{23869.295,8524.748,10.000,0.000},
				{24737.269,8679.744,10.000,0.000},
				{25620.743,8834.739,10.000,0.000},
				{26519.717,8989.734,10.000,0.000},
				{27434.190,9144.730,10.000,0.000},
				{28364.162,9299.725,10.000,0.000},
				{29309.634,9454.721,10.000,0.000},
				{30268.620,9589.852,10.000,0.000},
				{31227.605,9589.852,10.000,0.000},
				{32186.590,9589.852,10.000,0.000},
				{33145.575,9589.852,10.000,0.000},
				{34104.561,9589.852,10.000,0.000},
				{35048.046,9434.857,10.000,0.000},
				{35976.032,9279.862,10.000,0.000},
				{36888.519,9124.866,10.000,0.000},
				{37785.506,8969.871,10.000,0.000},
				{38666.994,8814.875,10.000,0.000},
				{39532.982,8659.880,10.000,0.000},
				{40383.470,8504.885,10.000,0.000},
				{41218.459,8349.889,10.000,0.000},
				{42037.948,8194.894,10.000,0.000},
				{42841.938,8039.898,10.000,0.000},
				{43630.429,7884.903,10.000,0.000},
				{44403.419,7729.907,10.000,0.000},
				{45160.910,7574.912,10.000,0.000},
				{45902.902,7419.917,10.000,0.000},
				{46629.394,7264.921,10.000,0.000},
				{47340.387,7109.926,10.000,0.000},
				{48035.880,6954.930,10.000,0.000},
				{48715.873,6799.935,10.000,0.000},
				{49380.367,6644.939,10.000,0.000},
				{50029.362,6489.944,10.000,0.000},
				{50662.857,6334.949,10.000,0.000},
				{51280.852,6179.953,10.000,0.000},
				{51883.348,6024.958,10.000,0.000},
				{52470.344,5869.962,10.000,0.000},
				{53041.841,5714.967,10.000,0.000},
				{53597.838,5559.972,10.000,0.000},
				{54138.335,5404.976,10.000,0.000},
				{54663.333,5249.981,10.000,0.000},
				{55172.832,5094.985,10.000,0.000},
				{55666.831,4939.990,10.000,0.000},
				{56145.330,4784.994,10.000,0.000},
				{56608.330,4629.999,10.000,0.000},
				{57055.831,4475.004,10.000,0.000},
				{57487.831,4320.008,10.000,0.000},
				{57904.333,4165.013,10.000,0.000},
				{57904.333,4165.013,10.000,0.000},
				{58291.821,3874.886,10.000,0.000},
				{58679.310,3874.886,10.000,0.010},
				{59066.798,3874.886,10.000,0.020},
				{59454.287,3874.886,10.000,0.040},
				{59841.775,3874.886,10.000,0.070},
				{60229.264,3874.886,10.000,0.100},
				{60616.753,3874.886,10.000,0.140},
				{61004.241,3874.886,10.000,0.190},
				{61391.730,3874.886,10.000,0.240},
				{61779.218,3874.886,10.000,0.300},
				{62166.707,3874.886,10.000,0.360},
				{62554.195,3874.886,10.000,0.430},
				{62941.684,3874.886,10.000,0.510},
				{63329.172,3874.886,10.000,0.600},
				{63716.661,3874.886,10.000,0.690},
				{64104.150,3874.886,10.000,0.790},
				{64491.638,3874.886,10.000,0.890},
				{64879.127,3874.886,10.000,1.000},
				{65266.615,3874.886,10.000,1.120},
				{65654.104,3874.886,10.000,1.240},
				{66041.592,3874.886,10.000,1.370},
				{66429.081,3874.886,10.000,1.510},
				{66816.569,3874.886,10.000,1.650},
				{67204.058,3874.886,10.000,1.800},
				{67591.547,3874.886,10.000,1.960},
				{67979.035,3874.886,10.000,2.130},
				{68366.524,3874.886,10.000,2.300},
				{68754.012,3874.886,10.000,2.470},
				{69141.501,3874.886,10.000,2.660},
				{69528.989,3874.886,10.000,2.850},
				{69916.478,3874.886,10.000,3.050},
				{70303.966,3874.886,10.000,3.260},
				{70691.455,3874.886,10.000,3.470},
				{71078.944,3874.886,10.000,3.700},
				{71466.432,3874.886,10.000,3.930},
				{71853.921,3874.886,10.000,4.170},
				{72241.409,3874.886,10.000,4.410},
				{72628.898,3874.886,10.000,4.670},
				{73016.386,3874.886,10.000,4.930},
				{73403.875,3874.886,10.000,5.200},
				{73791.363,3874.886,10.000,5.480},
				{74178.852,3874.886,10.000,5.770},
				{74566.341,3874.886,10.000,6.070},
				{74953.829,3874.886,10.000,6.370},
				{75341.318,3874.886,10.000,6.690},
				{75728.806,3874.886,10.000,7.010},
				{76116.295,3874.886,10.000,7.350},
				{76503.783,3874.886,10.000,7.690},
				{76891.272,3874.886,10.000,8.050},
				{77278.760,3874.886,10.000,8.410},
				{77666.249,3874.886,10.000,8.790},
				{78053.738,3874.886,10.000,9.170},
				{78441.226,3874.886,10.000,9.570},
				{78828.715,3874.886,10.000,9.980},
				{79216.203,3874.886,10.000,10.400},
				{79603.692,3874.886,10.000,10.830},
				{79991.180,3874.886,10.000,11.270},
				{80378.669,3874.886,10.000,11.720},
				{80766.157,3874.886,10.000,12.190},
				{81153.646,3874.886,10.000,12.670},
				{81541.135,3874.886,10.000,13.160},
				{81928.623,3874.886,10.000,13.660},
				{82316.112,3874.886,10.000,14.180},
				{82703.600,3874.886,10.000,14.710},
				{83091.089,3874.886,10.000,15.250},
				{83478.577,3874.886,10.000,15.810},
				{83866.066,3874.886,10.000,16.380},
				{84253.554,3874.886,10.000,16.960},
				{84641.043,3874.886,10.000,17.560},
				{85028.532,3874.886,10.000,18.170},
				{85416.020,3874.886,10.000,18.800},
				{85803.509,3874.886,10.000,19.440},
				{86190.997,3874.886,10.000,20.090},
				{86578.486,3874.886,10.000,20.760},
				{86965.974,3874.886,10.000,21.440},
				{87353.463,3874.886,10.000,22.140},
				{87740.951,3874.886,10.000,22.850},
				{88128.440,3874.886,10.000,23.580},
				{88515.929,3874.886,10.000,24.320},
				{88903.417,3874.886,10.000,25.070},
				{89290.906,3874.886,10.000,25.840},
				{89678.394,3874.886,10.000,26.620},
				{90065.883,3874.886,10.000,27.410},
				{90453.371,3874.886,10.000,28.220},
				{90840.860,3874.886,10.000,29.040},
				{91228.348,3874.886,10.000,29.870},
				{91615.837,3874.886,10.000,30.710},
				{92003.326,3874.886,10.000,31.560},
				{92390.814,3874.886,10.000,32.430},
				{92778.303,3874.886,10.000,33.300},
				{93165.791,3874.886,10.000,34.180},
				{93553.280,3874.886,10.000,35.070},
				{93940.768,3874.886,10.000,35.970},
				{94328.257,3874.886,10.000,36.870},
				{94715.745,3874.886,10.000,37.780},
				{95103.234,3874.886,10.000,38.700},
				{95490.723,3874.886,10.000,39.620},
				{95878.211,3874.886,10.000,40.540},
				{96265.700,3874.886,10.000,41.460},
				{96653.188,3874.886,10.000,42.390},
				{97040.677,3874.886,10.000,43.310},
				{97428.165,3874.886,10.000,44.240},
				{97815.654,3874.886,10.000,45.160},
				{98203.142,3874.886,10.000,46.080},
				{98590.631,3874.886,10.000,47.000},
				{98978.120,3874.886,10.000,47.910},
				{99365.608,3874.886,10.000,48.820},
				{99753.097,3874.886,10.000,49.710},
				{100140.585,3874.886,10.000,50.610},
				{100528.074,3874.886,10.000,51.490},
				{100915.562,3874.886,10.000,52.370},
				{101303.051,3874.886,10.000,53.230},
				{101690.540,3874.886,10.000,54.090},
				{102078.028,3874.886,10.000,54.930},
				{102465.517,3874.886,10.000,55.770},
				{102853.005,3874.886,10.000,56.590},
				{103240.494,3874.886,10.000,57.400},
				{103627.982,3874.886,10.000,58.200},
				{104015.471,3874.886,10.000,58.980},
				{104402.959,3874.886,10.000,59.750},
				{104790.448,3874.886,10.000,60.510},
				{105177.937,3874.886,10.000,61.250},
				{105565.425,3874.886,10.000,61.980},
				{105952.914,3874.886,10.000,62.690},
				{106340.402,3874.886,10.000,63.400},
				{106727.891,3874.886,10.000,64.080},
				{107115.379,3874.886,10.000,64.750},
				{107502.868,3874.886,10.000,65.410},
				{107890.356,3874.886,10.000,66.060},
				{108277.845,3874.886,10.000,66.690},
				{108665.334,3874.886,10.000,67.300},
				{109052.822,3874.886,10.000,67.900},
				{109440.311,3874.886,10.000,68.490},
				{109827.799,3874.886,10.000,69.060},
				{110215.288,3874.886,10.000,69.620},
				{110602.776,3874.886,10.000,70.170},
				{110990.265,3874.886,10.000,70.700},
				{111377.753,3874.886,10.000,71.220},
				{111765.242,3874.886,10.000,71.730},
				{112152.731,3874.886,10.000,72.230},
				{112540.219,3874.886,10.000,72.710},
				{112927.708,3874.886,10.000,73.180},
				{113315.196,3874.886,10.000,73.640},
				{113702.685,3874.886,10.000,74.090},
				{114090.173,3874.886,10.000,74.520},
				{114477.662,3874.886,10.000,74.940},
				{114865.150,3874.886,10.000,75.360},
				{115252.639,3874.886,10.000,75.760},
				{115640.128,3874.886,10.000,76.150},
				{116027.616,3874.886,10.000,76.530},
				{116415.105,3874.886,10.000,76.900},
				{116802.593,3874.886,10.000,77.260},
				{117190.082,3874.886,10.000,77.610},
				{117577.570,3874.886,10.000,77.950},
				{117965.059,3874.886,10.000,78.280},
				{118352.547,3874.886,10.000,78.610},
				{118740.036,3874.886,10.000,78.920},
				{119127.525,3874.886,10.000,79.230},
				{119515.013,3874.886,10.000,79.520},
				{119902.502,3874.886,10.000,79.810},
				{120289.990,3874.886,10.000,80.090},
				{120677.479,3874.886,10.000,80.360},
				{121064.967,3874.886,10.000,80.620},
				{121452.456,3874.886,10.000,80.880},
				{121839.944,3874.886,10.000,81.130},
				{122227.433,3874.886,10.000,81.370},
				{122614.922,3874.886,10.000,81.600},
				{123002.410,3874.886,10.000,81.830},
				{123389.899,3874.886,10.000,82.050},
				{123777.387,3874.886,10.000,82.260},
				{124164.876,3874.886,10.000,82.470},
				{124552.364,3874.886,10.000,82.660},
				{124939.853,3874.886,10.000,82.860},
				{125327.341,3874.886,10.000,83.040},
				{125714.830,3874.886,10.000,83.220},
				{126102.319,3874.886,10.000,83.400},
				{126489.807,3874.886,10.000,83.560},
				{126877.296,3874.886,10.000,83.730},
				{127264.784,3874.886,10.000,83.880},
				{127652.273,3874.886,10.000,84.030},
				{128039.761,3874.886,10.000,84.180},
				{128427.250,3874.886,10.000,84.320},
				{128814.738,3874.886,10.000,84.450},
				{129202.227,3874.886,10.000,84.580},
				{129589.716,3874.886,10.000,84.700},
				{129977.204,3874.886,10.000,84.820},
				{130364.693,3874.886,10.000,84.930},
				{130752.181,3874.886,10.000,85.040},
				{131139.670,3874.886,10.000,85.140},
				{131527.158,3874.886,10.000,85.230},
				{131914.647,3874.886,10.000,85.330},
				{132302.135,3874.886,10.000,85.410},
				{132689.624,3874.886,10.000,85.500},
				{133077.113,3874.886,10.000,85.570},
				{133464.601,3874.886,10.000,85.650},
				{133852.090,3874.886,10.000,85.710},
				{134239.578,3874.886,10.000,85.780},
				{134627.067,3874.886,10.000,85.840},
				{135014.555,3874.886,10.000,85.890},
				{135402.044,3874.886,10.000,85.940},
				{135789.532,3874.886,10.000,85.980},
				{136177.021,3874.886,10.000,86.020},
				{136564.510,3874.886,10.000,86.060},
				{136951.998,3874.886,10.000,86.090},
				{137339.487,3874.886,10.000,86.120},
				{137726.975,3874.886,10.000,86.140},
				{138114.464,3874.886,10.000,86.160},
				{138501.952,3874.886,10.000,86.170},
				{138889.441,3874.886,10.000,86.180},
				{139276.929,3874.886,10.000,86.190},
				{139664.418,3874.886,10.000,86.190},
				{140051.907,3874.886,10.000,86.180},
				{140439.395,3874.886,10.000,86.170},
				{140826.884,3874.886,10.000,86.160},
				{141214.372,3874.886,10.000,86.140},
				{141601.861,3874.886,10.000,86.120},
				{141989.349,3874.886,10.000,86.090},
				{142376.838,3874.886,10.000,86.060},
				{142764.326,3874.886,10.000,86.030},
				{143151.815,3874.886,10.000,85.990},
				{143539.304,3874.886,10.000,85.940},
				{143926.792,3874.886,10.000,85.900},
				{144314.281,3874.886,10.000,85.840},
				{144701.769,3874.886,10.000,85.780},
				{145089.258,3874.886,10.000,85.720},
				{145476.746,3874.886,10.000,85.650},
				{145864.235,3874.886,10.000,85.580},
				{146251.723,3874.886,10.000,85.510},
				{146639.212,3874.886,10.000,85.420},
				{147026.701,3874.886,10.000,85.340},
				{147414.189,3874.886,10.000,85.250},
				{147801.678,3874.886,10.000,85.150},
				{148189.166,3874.886,10.000,85.050},
				{148576.655,3874.886,10.000,84.940},
				{148964.143,3874.886,10.000,84.830},
				{149351.632,3874.886,10.000,84.710},
				{149739.120,3874.886,10.000,84.590},
				{150126.609,3874.886,10.000,84.470},
				{150514.098,3874.886,10.000,84.330},
				{150901.586,3874.886,10.000,84.190},
				{151289.075,3874.886,10.000,84.050},
				{151676.563,3874.886,10.000,83.900},
				{152064.052,3874.886,10.000,83.750},
				{152451.540,3874.886,10.000,83.590},
				{152839.029,3874.886,10.000,83.420},
				{153226.517,3874.886,10.000,83.250},
				{153614.006,3874.886,10.000,83.070},
				{154001.495,3874.886,10.000,82.880},
				{154388.983,3874.886,10.000,82.690},
				{154776.472,3874.886,10.000,82.490},
				{155163.960,3874.886,10.000,82.290},
				{155551.449,3874.886,10.000,82.080},
				{155938.937,3874.886,10.000,81.860},
				{156326.426,3874.886,10.000,81.630},
				{156713.914,3874.886,10.000,81.400},
				{157101.403,3874.886,10.000,81.160},
				{157488.892,3874.886,10.000,80.910},
				{157876.380,3874.886,10.000,80.660},
				{158263.869,3874.886,10.000,80.400},
				{158651.357,3874.886,10.000,80.120},
				{159038.846,3874.886,10.000,79.850},
				{159426.334,3874.886,10.000,79.560},
				{159813.823,3874.886,10.000,79.260},
				{160201.311,3874.886,10.000,78.960},
				{160588.800,3874.886,10.000,78.650},
				{160976.289,3874.886,10.000,78.330},
				{161363.777,3874.886,10.000,78.000},
				{161751.266,3874.886,10.000,77.660},
				{162138.754,3874.886,10.000,77.310},
				{162526.243,3874.886,10.000,76.950},
				{162913.731,3874.886,10.000,76.580},
				{163301.220,3874.886,10.000,76.200},
				{163688.708,3874.886,10.000,75.810},
				{164076.197,3874.886,10.000,75.410},
				{164463.686,3874.886,10.000,75.000},
				{164851.174,3874.886,10.000,74.580},
				{165238.663,3874.886,10.000,74.140},
				{165626.151,3874.886,10.000,73.700},
				{166013.640,3874.886,10.000,73.240},
				{166401.128,3874.886,10.000,72.770},
				{166788.617,3874.886,10.000,72.290},
				{167176.105,3874.886,10.000,71.800},
				{167563.594,3874.886,10.000,71.290},
				{167951.083,3874.886,10.000,70.770},
				{168338.571,3874.886,10.000,70.240},
				{168726.060,3874.886,10.000,69.690},
				{169113.548,3874.886,10.000,69.140},
				{169501.037,3874.886,10.000,68.560},
				{169888.525,3874.886,10.000,67.980},
				{170276.014,3874.886,10.000,67.380},
				{170663.502,3874.886,10.000,66.770},
				{171050.991,3874.886,10.000,66.140},
				{171438.480,3874.886,10.000,65.500},
				{171825.968,3874.886,10.000,64.840},
				{172213.457,3874.886,10.000,64.170},
				{172600.945,3874.886,10.000,63.480},
				{172988.434,3874.886,10.000,62.790},
				{173375.922,3874.886,10.000,62.070},
				{173763.411,3874.886,10.000,61.350},
				{174150.899,3874.886,10.000,60.600},
				{174538.388,3874.886,10.000,59.850},
				{174925.877,3874.886,10.000,59.080},
				{175313.365,3874.886,10.000,58.300},
				{175700.854,3874.886,10.000,57.500},
				{176088.342,3874.886,10.000,56.690},
				{176475.831,3874.886,10.000,55.870},
				{176863.319,3874.886,10.000,55.040},
				{177250.808,3874.886,10.000,54.200},
				{177638.296,3874.886,10.000,53.340},
				{178025.785,3874.886,10.000,52.480},
				{178413.274,3874.886,10.000,51.600},
				{178800.762,3874.886,10.000,50.720},
				{179188.251,3874.886,10.000,49.830},
				{179575.739,3874.886,10.000,48.930},
				{179963.228,3874.886,10.000,48.030},
				{180350.716,3874.886,10.000,47.120},
				{180738.205,3874.886,10.000,46.200},
				{181125.693,3874.886,10.000,45.280},
				{181513.182,3874.886,10.000,44.360},
				{181900.671,3874.886,10.000,43.430},
				{182288.159,3874.886,10.000,42.510},
				{182675.648,3874.886,10.000,41.580},
				{183063.136,3874.886,10.000,40.660},
				{183450.625,3874.886,10.000,39.740},
				{183838.113,3874.886,10.000,38.820},
				{184225.602,3874.886,10.000,37.900},
				{184613.090,3874.886,10.000,36.990},
				{185000.579,3874.886,10.000,36.090},
				{185388.068,3874.886,10.000,35.190},
				{185775.556,3874.886,10.000,34.300},
				{186163.045,3874.886,10.000,33.410},
				{186550.533,3874.886,10.000,32.540},
				{186938.022,3874.886,10.000,31.670},
				{187325.510,3874.886,10.000,30.820},
				{187712.999,3874.886,10.000,29.970},
				{188100.487,3874.886,10.000,29.140},
				{188487.976,3874.886,10.000,28.320},
				{188875.465,3874.886,10.000,27.510},
				{189262.953,3874.886,10.000,26.720},
				{189650.442,3874.886,10.000,25.940},
				{190037.930,3874.886,10.000,25.170},
				{190425.419,3874.886,10.000,24.410},
				{190812.907,3874.886,10.000,23.670},
				{191200.396,3874.886,10.000,22.940},
				{191587.884,3874.886,10.000,22.230},
				{191975.373,3874.886,10.000,21.530},
				{192362.862,3874.886,10.000,20.850},
				{192750.350,3874.886,10.000,20.180},
				{193137.839,3874.886,10.000,19.520},
				{193525.327,3874.886,10.000,18.880},
				{193912.816,3874.886,10.000,18.250},
				{194300.304,3874.886,10.000,17.640},
				{194687.793,3874.886,10.000,17.040},
				{195075.281,3874.886,10.000,16.450},
				{195462.770,3874.886,10.000,15.880},
				{195850.259,3874.886,10.000,15.320},
				{196237.747,3874.886,10.000,14.780},
				{196625.236,3874.886,10.000,14.250},
				{197012.724,3874.886,10.000,13.730},
				{197400.213,3874.886,10.000,13.220},
				{197787.701,3874.886,10.000,12.730},
				{198175.190,3874.886,10.000,12.250},
				{198562.678,3874.886,10.000,11.780},
				{198950.167,3874.886,10.000,11.330},
				{199337.656,3874.886,10.000,10.890},
				{199725.144,3874.886,10.000,10.450},
				{200112.633,3874.886,10.000,10.030},
				{200500.121,3874.886,10.000,9.620},
				{200887.610,3874.886,10.000,9.230},
				{201275.098,3874.886,10.000,8.840},
				{201662.587,3874.886,10.000,8.460},
				{202050.075,3874.886,10.000,8.100},
				{202437.564,3874.886,10.000,7.740},
				{202825.053,3874.886,10.000,7.390},
				{203212.541,3874.886,10.000,7.060},
				{203600.030,3874.886,10.000,6.730},
				{203987.518,3874.886,10.000,6.410},
				{204375.007,3874.886,10.000,6.110},
				{204762.495,3874.886,10.000,5.810},
				{205149.984,3874.886,10.000,5.520},
				{205537.472,3874.886,10.000,5.240},
				{205924.961,3874.886,10.000,4.960},
				{206312.450,3874.886,10.000,4.700},
				{206699.938,3874.886,10.000,4.440},
				{207087.427,3874.886,10.000,4.200},
				{207474.915,3874.886,10.000,3.960},
				{207862.404,3874.886,10.000,3.730},
				{208249.892,3874.886,10.000,3.500},
				{208637.381,3874.886,10.000,3.290},
				{209024.869,3874.886,10.000,3.080},
				{209412.358,3874.886,10.000,2.880},
				{209799.847,3874.886,10.000,2.680},
				{210187.335,3874.886,10.000,2.500},
				{210574.824,3874.886,10.000,2.320},
				{210962.312,3874.886,10.000,2.150},
				{211349.801,3874.886,10.000,1.980},
				{211737.289,3874.886,10.000,1.820},
				{212124.778,3874.886,10.000,1.670},
				{212512.266,3874.886,10.000,1.530},
				{212899.755,3874.886,10.000,1.390},
				{213287.244,3874.886,10.000,1.260},
				{213674.732,3874.886,10.000,1.130},
				{214062.221,3874.886,10.000,1.020},
				{214449.709,3874.886,10.000,0.900},
				{214837.198,3874.886,10.000,0.800},
				{215224.686,3874.886,10.000,0.700},
				{215612.175,3874.886,10.000,0.610},
				{215999.663,3874.886,10.000,0.520},
				{216387.152,3874.886,10.000,0.440},
				{216759.141,3719.890,10.000,0.370},
				{217115.630,3564.895,10.000,0.310},
				{217456.620,3409.899,10.000,0.260},
				{217782.111,3254.904,10.000,0.210},
				{218092.102,3099.908,10.000,0.170},
				{218386.593,2944.913,10.000,0.140},
				{218665.585,2789.918,10.000,0.110},
				{218929.077,2634.922,10.000,0.080},
				{219177.070,2479.927,10.000,0.060},
				{219409.563,2324.931,10.000,0.050},
				{219626.556,2169.936,10.000,0.030},
				{219828.050,2014.940,10.000,0.020},
				{220014.045,1859.945,10.000,0.020},
				{220184.540,1704.950,10.000,0.010},
				{220339.535,1549.954,10.000,0.010},
				{220479.031,1394.959,10.000,0.000},
				{220603.027,1239.963,10.000,0.000},
				{220711.524,1084.968,10.000,0.000},
				{220804.522,929.973,10.000,0.000},
				{220882.019,774.977,10.000,0.000},
				{220944.017,619.982,10.000,0.000},
				{220990.516,464.986,10.000,0.000},
				{221021.515,309.991,10.000,0.000}		};

}