package frc.arcs;

import com.team319.follower.SrxMotionProfile;
import com.team319.follower.SrxTrajectory;

public class StraightBack10FeetArc extends SrxTrajectory {
	
	// WAYPOINTS:
	// (X,Y,degrees)
	// (1.63,3.79,0.00)
	// (11.63,3.79,0.00)
	
    public StraightBack10FeetArc() {
		super();
		this.highGear = true;
		centerProfile = new SrxMotionProfile(centerPoints.length, centerPoints);
	}

	
    public StraightBack10FeetArc(boolean flipped) {
		super();
		this.highGear = true;
		this.flipped = flipped;
		centerProfile = new SrxMotionProfile(centerPoints.length, centerPoints);
	}

	public boolean highGear = true;

	double[][] centerPoints = {
				{-0.000,-0.000,10.000,0.000},
				{-0.271,-2.714,10.000,0.000},
				{-0.814,-5.427,10.000,0.000},
				{-1.628,-8.141,10.000,0.000},
				{-2.714,-10.854,10.000,0.000},
				{-4.070,-13.568,10.000,0.000},
				{-5.698,-16.281,10.000,0.000},
				{-7.598,-18.995,10.000,0.000},
				{-9.769,-21.708,10.000,0.000},
				{-12.211,-24.422,10.000,0.000},
				{-14.924,-27.135,10.000,0.000},
				{-17.909,-29.849,10.000,0.000},
				{-21.166,-32.562,10.000,0.000},
				{-24.693,-35.276,10.000,0.000},
				{-28.492,-37.989,10.000,0.000},
				{-32.562,-40.703,10.000,0.000},
				{-36.904,-43.416,10.000,0.000},
				{-41.517,-46.130,10.000,0.000},
				{-46.401,-48.844,10.000,0.000},
				{-51.557,-51.557,10.000,0.000},
				{-56.984,-54.271,10.000,0.000},
				{-62.682,-56.984,10.000,0.000},
				{-68.652,-59.698,10.000,0.000},
				{-74.893,-62.411,10.000,0.000},
				{-81.406,-65.125,10.000,0.000},
				{-88.190,-67.838,10.000,0.000},
				{-95.245,-70.552,10.000,0.000},
				{-102.571,-73.265,10.000,0.000},
				{-110.169,-75.979,10.000,0.000},
				{-118.038,-78.692,10.000,0.000},
				{-126.179,-81.406,10.000,0.000},
				{-134.591,-84.119,10.000,0.000},
				{-143.274,-86.833,10.000,0.000},
				{-152.229,-89.546,10.000,0.000},
				{-161.455,-92.260,10.000,0.000},
				{-170.952,-94.973,10.000,0.000},
				{-180.721,-97.687,10.000,0.000},
				{-190.761,-100.401,10.000,0.000},
				{-201.072,-103.114,10.000,0.000},
				{-211.655,-105.828,10.000,0.000},
				{-222.509,-108.541,10.000,0.000},
				{-233.635,-111.255,10.000,0.000},
				{-245.032,-113.968,10.000,0.000},
				{-256.700,-116.682,10.000,0.000},
				{-268.639,-119.395,10.000,0.000},
				{-280.850,-122.109,10.000,0.000},
				{-293.332,-124.822,10.000,0.000},
				{-306.086,-127.536,10.000,0.000},
				{-319.111,-130.249,10.000,0.000},
				{-332.407,-132.963,10.000,0.000},
				{-345.975,-135.676,10.000,0.000},
				{-359.814,-138.390,10.000,0.000},
				{-373.924,-141.103,10.000,0.000},
				{-388.306,-143.817,10.000,0.000},
				{-402.959,-146.531,10.000,0.000},
				{-417.883,-149.244,10.000,0.000},
				{-433.079,-151.958,10.000,0.000},
				{-448.546,-154.671,10.000,0.000},
				{-464.285,-157.385,10.000,0.000},
				{-480.294,-160.098,10.000,0.000},
				{-496.576,-162.812,10.000,0.000},
				{-513.128,-165.525,10.000,0.000},
				{-529.952,-168.239,10.000,0.000},
				{-547.047,-170.952,10.000,0.000},
				{-564.414,-173.666,10.000,0.000},
				{-582.052,-176.379,10.000,0.000},
				{-599.961,-179.093,10.000,0.000},
				{-618.142,-181.806,10.000,0.000},
				{-636.594,-184.520,10.000,0.000},
				{-655.317,-187.233,10.000,0.000},
				{-674.312,-189.947,10.000,0.000},
				{-693.578,-192.660,10.000,0.000},
				{-713.115,-195.374,10.000,0.000},
				{-732.924,-198.088,10.000,0.000},
				{-753.004,-200.801,10.000,0.000},
				{-773.356,-203.515,10.000,0.000},
				{-793.978,-206.228,10.000,0.000},
				{-814.872,-208.942,10.000,0.000},
				{-836.038,-211.655,10.000,0.000},
				{-857.475,-214.369,10.000,0.000},
				{-879.183,-217.082,10.000,0.000},
				{-901.163,-219.796,10.000,0.000},
				{-923.414,-222.509,10.000,0.000},
				{-945.936,-225.223,10.000,0.000},
				{-968.730,-227.936,10.000,0.000},
				{-991.795,-230.650,10.000,0.000},
				{-1015.131,-233.363,10.000,0.000},
				{-1038.739,-236.077,10.000,0.000},
				{-1062.618,-238.790,10.000,0.000},
				{-1086.768,-241.504,10.000,0.000},
				{-1111.190,-244.218,10.000,0.000},
				{-1135.883,-246.931,10.000,0.000},
				{-1160.847,-249.645,10.000,0.000},
				{-1186.083,-252.358,10.000,0.000},
				{-1211.590,-255.072,10.000,0.000},
				{-1237.369,-257.785,10.000,0.000},
				{-1263.419,-260.499,10.000,0.000},
				{-1289.740,-263.212,10.000,0.000},
				{-1316.332,-265.926,10.000,0.000},
				{-1343.196,-268.639,10.000,0.000},
				{-1370.332,-271.353,10.000,0.000},
				{-1397.467,-271.353,10.000,0.000},
				{-1424.602,-271.353,10.000,0.000},
				{-1451.738,-271.353,10.000,0.000},
				{-1478.873,-271.353,10.000,0.000},
				{-1506.008,-271.353,10.000,0.000},
				{-1533.143,-271.353,10.000,0.000},
				{-1560.279,-271.353,10.000,0.000},
				{-1587.414,-271.353,10.000,0.000},
				{-1614.549,-271.353,10.000,0.000},
				{-1641.685,-271.353,10.000,0.000},
				{-1668.820,-271.353,10.000,0.000},
				{-1695.955,-271.353,10.000,0.000},
				{-1723.090,-271.353,10.000,0.000},
				{-1750.226,-271.353,10.000,0.000},
				{-1777.361,-271.353,10.000,0.000},
				{-1804.496,-271.353,10.000,0.000},
				{-1831.631,-271.353,10.000,0.000},
				{-1858.767,-271.353,10.000,0.000},
				{-1885.902,-271.353,10.000,0.000},
				{-1913.037,-271.353,10.000,0.000},
				{-1940.173,-271.353,10.000,0.000},
				{-1967.308,-271.353,10.000,0.000},
				{-1994.443,-271.353,10.000,0.000},
				{-2021.578,-271.353,10.000,0.000},
				{-2048.714,-271.353,10.000,0.000},
				{-2075.849,-271.353,10.000,0.000},
				{-2102.984,-271.353,10.000,0.000},
				{-2130.120,-271.353,10.000,0.000},
				{-2157.255,-271.353,10.000,0.000},
				{-2184.390,-271.353,10.000,0.000},
				{-2211.525,-271.353,10.000,0.000},
				{-2238.661,-271.353,10.000,0.000},
				{-2265.796,-271.353,10.000,0.000},
				{-2292.931,-271.353,10.000,0.000},
				{-2320.067,-271.353,10.000,0.000},
				{-2347.202,-271.353,10.000,0.000},
				{-2374.337,-271.353,10.000,0.000},
				{-2401.472,-271.353,10.000,0.000},
				{-2428.608,-271.353,10.000,0.000},
				{-2455.743,-271.353,10.000,0.000},
				{-2482.878,-271.353,10.000,0.000},
				{-2510.014,-271.353,10.000,0.000},
				{-2537.149,-271.353,10.000,0.000},
				{-2564.284,-271.353,10.000,0.000},
				{-2591.419,-271.353,10.000,0.000},
				{-2618.555,-271.353,10.000,0.000},
				{-2645.690,-271.353,10.000,0.000},
				{-2672.825,-271.353,10.000,0.000},
				{-2699.960,-271.353,10.000,0.000},
				{-2727.096,-271.353,10.000,0.000},
				{-2754.231,-271.353,10.000,0.000},
				{-2781.366,-271.353,10.000,0.000},
				{-2808.502,-271.353,10.000,0.000},
				{-2835.637,-271.353,10.000,0.000},
				{-2862.772,-271.353,10.000,0.000},
				{-2889.907,-271.353,10.000,0.000},
				{-2917.043,-271.353,10.000,0.000},
				{-2944.178,-271.353,10.000,0.000},
				{-2971.313,-271.353,10.000,0.000},
				{-2998.449,-271.353,10.000,0.000},
				{-3025.584,-271.353,10.000,0.000},
				{-3052.719,-271.353,10.000,0.000},
				{-3079.854,-271.353,10.000,0.000},
				{-3106.990,-271.353,10.000,0.000},
				{-3134.125,-271.353,10.000,0.000},
				{-3161.260,-271.353,10.000,0.000},
				{-3188.396,-271.353,10.000,0.000},
				{-3215.531,-271.353,10.000,0.000},
				{-3242.666,-271.353,10.000,0.000},
				{-3269.801,-271.353,10.000,0.000},
				{-3296.937,-271.353,10.000,0.000},
				{-3324.072,-271.353,10.000,0.000},
				{-3351.207,-271.353,10.000,0.000},
				{-3378.343,-271.353,10.000,0.000},
				{-3405.478,-271.353,10.000,0.000},
				{-3432.613,-271.353,10.000,0.000},
				{-3459.748,-271.353,10.000,0.000},
				{-3486.884,-271.353,10.000,0.000},
				{-3514.019,-271.353,10.000,0.000},
				{-3541.154,-271.353,10.000,0.000},
				{-3568.289,-271.353,10.000,0.000},
				{-3595.425,-271.353,10.000,0.000},
				{-3622.560,-271.353,10.000,0.000},
				{-3649.695,-271.353,10.000,0.000},
				{-3676.831,-271.353,10.000,0.000},
				{-3703.966,-271.353,10.000,0.000},
				{-3731.101,-271.353,10.000,0.000},
				{-3758.236,-271.353,10.000,0.000},
				{-3785.372,-271.353,10.000,0.000},
				{-3812.507,-271.353,10.000,0.000},
				{-3839.642,-271.353,10.000,0.000},
				{-3866.778,-271.353,10.000,0.000},
				{-3893.913,-271.353,10.000,0.000},
				{-3921.048,-271.353,10.000,0.000},
				{-3948.183,-271.353,10.000,0.000},
				{-3975.319,-271.353,10.000,0.000},
				{-4002.454,-271.353,10.000,0.000},
				{-4029.589,-271.353,10.000,0.000},
				{-4056.725,-271.353,10.000,0.000},
				{-4083.860,-271.353,10.000,0.000},
				{-4110.724,-268.639,10.000,0.000},
				{-4137.316,-265.926,10.000,0.000},
				{-4163.638,-263.212,10.000,0.000},
				{-4189.687,-260.499,10.000,0.000},
				{-4215.466,-257.785,10.000,0.000},
				{-4240.973,-255.072,10.000,0.000},
				{-4266.209,-252.358,10.000,0.000},
				{-4291.173,-249.645,10.000,0.000},
				{-4315.866,-246.931,10.000,0.000},
				{-4340.288,-244.218,10.000,0.000},
				{-4364.439,-241.504,10.000,0.000},
				{-4388.318,-238.790,10.000,0.000},
				{-4411.925,-236.077,10.000,0.000},
				{-4435.262,-233.363,10.000,0.000},
				{-4458.327,-230.650,10.000,0.000},
				{-4481.120,-227.936,10.000,0.000},
				{-4503.643,-225.223,10.000,0.000},
				{-4525.894,-222.509,10.000,0.000},
				{-4547.873,-219.796,10.000,0.000},
				{-4569.581,-217.082,10.000,0.000},
				{-4591.018,-214.369,10.000,0.000},
				{-4612.184,-211.655,10.000,0.000},
				{-4633.078,-208.942,10.000,0.000},
				{-4653.701,-206.228,10.000,0.000},
				{-4674.052,-203.515,10.000,0.000},
				{-4694.132,-200.801,10.000,0.000},
				{-4713.941,-198.088,10.000,0.000},
				{-4733.478,-195.374,10.000,0.000},
				{-4752.744,-192.660,10.000,0.000},
				{-4771.739,-189.947,10.000,0.000},
				{-4790.463,-187.233,10.000,0.000},
				{-4808.915,-184.520,10.000,0.000},
				{-4827.095,-181.806,10.000,0.000},
				{-4845.004,-179.093,10.000,0.000},
				{-4862.642,-176.379,10.000,0.000},
				{-4880.009,-173.666,10.000,0.000},
				{-4897.104,-170.952,10.000,0.000},
				{-4913.928,-168.239,10.000,0.000},
				{-4930.481,-165.525,10.000,0.000},
				{-4946.762,-162.812,10.000,0.000},
				{-4962.772,-160.098,10.000,0.000},
				{-4978.510,-157.385,10.000,0.000},
				{-4993.977,-154.671,10.000,0.000},
				{-5009.173,-151.958,10.000,0.000},
				{-5024.097,-149.244,10.000,0.000},
				{-5038.750,-146.531,10.000,0.000},
				{-5053.132,-143.817,10.000,0.000},
				{-5067.242,-141.103,10.000,0.000},
				{-5081.081,-138.390,10.000,0.000},
				{-5094.649,-135.676,10.000,0.000},
				{-5107.945,-132.963,10.000,0.000},
				{-5120.970,-130.249,10.000,0.000},
				{-5133.724,-127.536,10.000,0.000},
				{-5146.206,-124.822,10.000,0.000},
				{-5158.417,-122.109,10.000,0.000},
				{-5170.356,-119.395,10.000,0.000},
				{-5182.025,-116.682,10.000,0.000},
				{-5193.421,-113.968,10.000,0.000},
				{-5204.547,-111.255,10.000,0.000},
				{-5215.401,-108.541,10.000,0.000},
				{-5225.984,-105.828,10.000,0.000},
				{-5236.295,-103.114,10.000,0.000},
				{-5246.335,-100.401,10.000,0.000},
				{-5256.104,-97.687,10.000,0.000},
				{-5265.601,-94.973,10.000,0.000},
				{-5274.827,-92.260,10.000,0.000},
				{-5283.782,-89.546,10.000,0.000},
				{-5292.465,-86.833,10.000,0.000},
				{-5300.877,-84.119,10.000,0.000},
				{-5309.018,-81.406,10.000,0.000},
				{-5316.887,-78.692,10.000,0.000},
				{-5324.485,-75.979,10.000,0.000},
				{-5331.811,-73.265,10.000,0.000},
				{-5338.867,-70.552,10.000,0.000},
				{-5345.650,-67.838,10.000,0.000},
				{-5352.163,-65.125,10.000,0.000},
				{-5358.404,-62.411,10.000,0.000},
				{-5364.374,-59.698,10.000,0.000},
				{-5370.072,-56.984,10.000,0.000},
				{-5375.499,-54.271,10.000,0.000},
				{-5380.655,-51.557,10.000,0.000},
				{-5385.539,-48.844,10.000,0.000},
				{-5390.152,-46.130,10.000,0.000},
				{-5394.494,-43.416,10.000,0.000},
				{-5398.564,-40.703,10.000,0.000},
				{-5402.363,-37.989,10.000,0.000},
				{-5405.891,-35.276,10.000,0.000},
				{-5409.147,-32.562,10.000,0.000},
				{-5412.132,-29.849,10.000,0.000},
				{-5414.845,-27.135,10.000,0.000},
				{-5417.288,-24.422,10.000,0.000},
				{-5419.458,-21.708,10.000,0.000},
				{-5421.358,-18.995,10.000,0.000},
				{-5422.986,-16.281,10.000,0.000},
				{-5424.343,-13.568,10.000,0.000},
				{-5425.428,-10.854,10.000,0.000},
				{-5426.242,-8.141,10.000,0.000},
				{-5426.785,-5.427,10.000,0.000}		};

}