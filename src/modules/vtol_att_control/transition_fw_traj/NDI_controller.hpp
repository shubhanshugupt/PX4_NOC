/****************************************************************************
 * Date: July 5, 2024
 * Author: Shubhanshu Gupta
 *
 * Outer loop trajectory control for transition of tailsitter vehicle
****************************************************************************/
#include <math.h>
#include <iostream>
#include <vector>
#include <matrix/math.hpp>
#include <Eigen/Eigen>
#include <Eigen/Dense>

#include <uORB/Subscription.hpp>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_local_position.h>

#include "params.hpp"

uORB::Subscription	_vehicle_attitude_sub{ORB_ID(vehicle_attitude)};
uORB::Subscription 	_local_pos_sub{ORB_ID(vehicle_local_position)};			// sensor subscription

vehicle_attitude_s		_v_att{};
vehicle_local_position_s	_local_pos{};

double t_prev = 0;
float pert = 0.001f;
double pert2 = (double) pert;
matrix::Vector3f vel_prev(0,0,0);
matrix::Vector3f eV_int(0,0,0);

matrix::Vector3f aerodyn_forces(double theta, double alpha, double V);
float pitch_sp_trans_f(float t, float trans_duration);	// returns pitch_sp(time) for forward transition
std::vector<double> NDI_cont(double time, double trans_duration, float pos_0[3], double yaw);
std::vector<double> OptimalControl(double time,  float pos_0[3]);
matrix::Vector3f dynamics(matrix::Vector3f X, matrix::Vector2f Control);
float hamilton(matrix::Vector3f X, matrix::Vector2f Control, matrix::Vector3f lambda);
matrix::Vector3d dynamics2(matrix::Vector3f X, matrix::Vector2f Control);
matrix::Vector3d aerodyn_forces2(double theta, double alpha, double V);

double hamilton2(matrix::Vector3f X, matrix::Vector2f Control, matrix::Vector3f lambda);

// Function to calculate pseudo-inverse
Eigen::MatrixXd pseudoInverse(const Eigen::MatrixXd& matrix, double tolerance = 1e-9) {
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(matrix, Eigen::ComputeThinU | Eigen::ComputeThinV);
    Eigen::VectorXd singularValues = svd.singularValues();
    Eigen::MatrixXd singularValuesInv = singularValues.asDiagonal();

    for (long i = 0; i < singularValues.size(); ++i) {
        if (singularValues(i) > tolerance) {
            singularValuesInv(i, i) = 1.0 / singularValues(i);
        } else {
            singularValuesInv(i, i) = 0.0;
        }
    }
    return svd.matrixV() * singularValuesInv * svd.matrixU().transpose();
}
double pitch_traj[36] = {0,0.053671342581157,0.167638898312111,0.275651790126659,0.355975039305876,0.410468228252778,0.449005413395409,0.481270837537753,0.513900989558263,0.550342293955607,0.591743414085322,0.637998615874762,0.688580214653797,0.743091088869123,0.80158249992209,0.864676604875897,0.933463972093396,1.00907620613855,1.09184883671383,1.18019950202948,1.26978442993962,1.35385841696591,1.42531731214932,1.47942819512082,1.51527610042305,1.53504791901305,1.54216305136945,1.53971986405289,1.52989095715269,1.51415544692175,1.49409039335187,1.4724710259608,1.45412261835663,1.44512624132717,1.44824697920268,1.45247118388072};


double ref_pos[3][36] = {
	{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
	{0,0.000543295223823957,0.00818906554685767,0.0357872475348871,0.0952021475861896,0.19469646405959,0.339318828119072,0.531956757487786,0.77434364401144,1.067737718739,1.41325994394006,1.81199192312312,2.2649439550569,2.77297898514352,3.33674430747144,3.95663057059137,4.63275048954864,5.36490848979057,6.15252548314889,6.99451011367512,7.88912449754385,8.83387559014339,9.82539668232605,10.8596089788466,11.932383316133,13.0401689187348,14.1802859664484,15.3508747522095,16.5506417247287,17.7785973887211,19.0338628429613,20.3155374246937,21.6226374256331,22.9541781059764,24.309457040734,25.688370733184},
	{-0,-0.0215510368599078,-0.0660933684314217,-0.131983356837445,-0.215516303530105,-0.312417605944665,-0.418971959171394,-0.532276302886952,-0.649967212224479,-0.769826524371499,-0.889501965564347,-1.00637723374965,-1.11754763431478,-1.21984832852235,-1.30989386152311,-1.38410747410666,-1.43875337721047,-1.47005190038268,-1.47457299438062,-1.45023303654547,-1.3981024473876,-1.32406246780754,-1.23677267964614,-1.14066952609091,-1.03348927429569,-0.912135219357415,-0.777244402061421,-0.633179622081336,-0.486444409245422,-0.344415651806636,-0.214622731925463,-0.104199473494825,-0.0190488084917087,0.0376817286445535,0.067824563528148,0.0787922965720128}
};

double ref_vel[3][36] = {
	{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
	{0,0.0216631272919633,0.153927960693804,0.418559535070845,0.784253379072626,1.21477190734486,1.68307358073565,2.17308323048392,2.67729323685861,3.19312484146879,3.71998534202996,4.25744984664342,4.80444681164585,5.35913592306454,5.91913582460847,6.48180137735709,7.04429999112634,7.60331079838212,8.15439820166145,8.69155671712832,9.20745168258409,9.69356981969067,10.1409275674072,10.5454515446253,10.9106814271813,11.2447225955945,11.5566092800017,11.8533684940794,12.1391865142035,12.4159948209297,12.6841985114599,12.9432478621878,13.1925700161691,13.4333043785962,13.6696811823544,13.9066752768502},
	{-0.1,-0.330701499075611,-0.557584013710344,-0.755625729888922,-0.910292294048656,-1.02369079186012,-1.10419955498064,-1.15921058224017,-1.19203574745331,-1.20241034132906,-1.18808263686634,-1.14614156857679,-1.07379238568544,-0.968645521289001,-0.82867639663151,-0.652104960142334,-0.43766395035644,-0.186150820574219,0.0953504643294236,0.385718435894706,0.642993120046135,0.819033370099686,0.916954391709796,1.00925946638407,1.14013701888463,1.28602514260833,1.40445493402078,1.46592607781419,1.4564615222164,1.37159956289761,1.21243442655187,0.986219916498861,0.711408872485469,0.426046629365843,0.190298438372882,0.041681493855921}
};

double d_ref_vel[3][36] = {
	{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
	{9.72545885949246e-06,0.651736735743176,2.00166994008992,3.19136566338357,4.00797581404225,4.50740058319398,4.79214200703917,4.96144649760074,5.08265120143834,5.18942199132573,5.29126475887078,5.38446101916769,5.4606350485886,5.51179358988421,5.53208277305963,5.5169736008694,5.46077437866831,5.35436298664037,5.18717790768984,4.9564602445924,4.67198732245615,4.33809903054461,3.96956584415357,3.62399291891739,3.34370045186543,3.14440862426319,3.0167869384015,2.93665005739424,2.88087914691382,2.83112557890809,2.77028902832831,2.68308050305577,2.56767466094551,2.45506060948382,2.42132846102743,0},
	{-2.31211997436833,-2.33162961612494,-2.11355788437707,-1.69131433455085,-1.25976183058317,-0.904584073508277,-0.631740284540358,-0.404927626014429,-0.184556923692936,0.0544006134477897,0.322366027999307,0.620286412109028,0.944827126495525,1.29229672443858,1.66060803552357,2.04772763601238,2.44326064694611,2.80800775971369,3.03932423110082,2.94419237974863,2.32214861065469,1.34887971024384,0.84571001046838,1.12081740671032,1.45844827598047,1.38565041593318,0.933746437726953,0.272783443185398,-0.471377316997355,-1.23080454914915,-1.95448676049227,-2.56088217075459,-2.89618452656018,-2.72028829955347,-1.90464667659496,-0}
};

float state[3][16] = {
	{0,-0.047416,-0.18809,-0.40752,-0.68662,-1.0006,-1.317,-1.5935,-1.7767,-1.8093,-1.6591,-1.342,-0.91477,-0.46621,-0.114,0.019366},
	{0,0.85215,1.7235,2.6218,3.5546,4.5292,5.5503,6.6146,7.6948,8.7047,9.5747,10.368,11.164,12.013,12.913,13.78},
	{0.01,-0.41639,-0.78953,-1.092,-1.3024,-1.3932,-1.327,-1.0557,-0.53042,0.24294,1.0452,1.6658,1.963,1.8102,1.1192,0.0046609}
};

float costate[3][16] = {
	{7.7465,7.7465,7.7465,7.7465,7.7465,7.7465,7.7465,7.7465,7.7465,7.7465,7.7465,7.7465,7.7465,7.7465,7.7465,7.7465},
	{-22.384,-23.088,-24.493,-26.612,-29.464,-33.032,-37.182,-41.507,-45.206,-48.297,-51.621,-55.848,-61.564,-69.108,-78.128,-87.818},
	{37.033,35.35,33.711,31.985,29.971,27.363,23.734,18.648,12.436,6.6147,1.528,-2.5762,-5.0843,-4.9011,-0.98367,1.8644}
};

float control[2][16] = {
	{33.194,32.777,32.298,31.706,30.917,29.804,28.169,25.687,21.79,17.946,15.803,15.004,15.408,16.893,18.851,18.851},
	{0.30574,0.32794,0.36544,0.41932,0.49136,0.58443,0.70285,0.85147,1.0307,1.1903,1.2988,1.3679,1.4019,1.4043,1.3865,1.3865}
};

float Time[16] = {0,0.23333,0.46667,0.7,0.93333,1.1667,1.4,1.6333,1.8667,2.1,2.3333,2.5667,2.8,3.0333,3.2667,3.5};

// std::vector<float> Tau = {-1.0,-0.96957,-0.8992,-0.79201,-0.65239,-0.48606,-0.29983,-0.10133,0.10133,0.29983,0.48606,0.65239,0.79201,0.8992,0.96957,1};

// float D_mat[16][16] = {
// 	{-60,81.172,-32.493,18.565,-12.368,8.9798,-6.8856,5.478,-4.47,3.709,-3.1056,2.6018,-2.1548,1.7245,-1.2542,0.5},
// 	{-13.302,0,18.842,-8.8037,5.4871,-3.864,2.9141,-2.2953,1.861,-1.5375,1.2835,-1.073,0.88738,-0.7095,0.51569,-0.20554},
// 	{3.029,-10.718,0,10.999,-5.3184,3.4107,-2.4559,1.8838,-1.5023,1.2276,-1.0172,0.846,-0.69712,0.55605,-0.40359,0.16076},
// 	{-1.2451,3.6028,-7.9128,0,7.9743,-3.9065,2.5367,-1.8458,1.4271,-1.1435,0.93514,-0.77082,0.63131,-0.50153,0.36315,-0.14451},
// 	{0.66914,-1.8115,3.0867,-6.433,0,6.4539,-3.1807,2.0779,-1.5192,1.1777,-0.94293,0.76641,-0.62183,0.491,-0.35425,0.14077},
// 	{-0.42161,1.107,-1.7178,2.7348,-5.6007,0,5.6094,-2.7726,1.816,-1.3292,1.0287,-0.81827,0.65466,-0.51231,0.36771,-0.14581},
// 	{0.29625,-0.76505,1.1335,-1.6274,2.5294,-5.1403,0,5.1441,-2.5454,1.6676,-1.2181,0.9365,-0.73358,0.56659,-0.40364,0.15958},
// 	{-0.22604,0.57793,-0.83385,1.1357,-1.5848,2.4367,-4.9335,0,4.9346,-2.4412,1.596,-1.1587,0.87804,-0.66496,0.46856,-0.18444},
// 	{0.18444,-0.46856,0.66496,-0.87804,1.1587,-1.596,2.4412,-4.9346,0,4.9335,-2.4367,1.5848,-1.1357,0.83385,-0.57793,0.22604},
// 	{-0.15958,0.40364,-0.56659,0.73358,-0.9365,1.2181,-1.6676,2.5454,-5.1441,0,5.1403,-2.5294,1.6274,-1.1335,0.76505,-0.29625},
// 	{0.14581,-0.36771,0.51231,-0.65466,0.81827,-1.0287,1.3292,-1.816,2.7726,-5.6094,0,5.6007,-2.7348,1.7178,-1.107,0.42161},
// 	{-0.14077,0.35425,-0.491,0.62183,-0.76641,0.94293,-1.1777,1.5192,-2.0779,3.1807,-6.4539,0,6.433,-3.0867,1.8115,-0.66914},
// 	{0.14451,-0.36315,0.50153,-0.63131,0.77082,-0.93514,1.1435,-1.4271,1.8458,-2.5367,3.9065,-7.9743,0,7.9128,-3.6028,1.2451},
// 	{-0.16076,0.40359,-0.55605,0.69712,-0.846,1.0172,-1.2276,1.5023,-1.8838,2.4559,-3.4107,5.3184,-10.999,0,10.718,-3.029},
// 	{0.20554,-0.51569,0.7095,-0.88738,1.073,-1.2835,1.5375,-1.861,2.2953,-2.9141,3.864,-5.4871,8.8037,-18.842,0,13.302},
// 	{-0.5,1.2542,-1.7245,2.1548,-2.6018,3.1056,-3.709,4.47,-5.478,6.8856,-8.9798,12.368,-18.565,32.493,-81.172,60}
// };

std::vector<float> Tau = {-1,-0.44721,0.44721,1};

float D_mat[4][4] = {
	{-3,4.0451,-1.5451,0.5},
	{-0.80902,0,1.118,-0.30902},
	{0.30902,-1.118,0,0.80902},
	{-0.5,1.5451,-4.0451,3}
};

// std::vector<float> Tau = {-1,-0.65465,0,0.65465,1};

// float D_mat[5][5] = {
// 	{-5,6.7565,-2.6667,1.4102,-0.5},
// 	{-1.241,0,1.7457,-0.76376,0.25901},
// 	{0.375,-1.3366,0,1.3366,-0.375},
// 	{-0.25901,0.76376,-1.7457,0,1.241},
// 	{0.5,-1.4102,2.6667,-6.7565,5}
// };

// std::vector<float> Tau = {-1,-0.76506,-0.28523,0.28523,0.76506,1};

// float D_mat[6][6] = {
// 	{-7.5,10.141,-4.0362,2.2447,-1.3499,0.5},
// 	{-1.7864,0,2.5234,-1.1528,0.65355,-0.23778},
// 	{0.48495,-1.7213,0,1.753,-0.78636,0.2697},
// 	{-0.2697,0.78636,-1.753,0,1.7213,-0.48495},
// 	{0.23778,-0.65355,1.1528,-2.5234,0,1.7864},
// 	{-0.5,1.3499,-2.2447,4.0362,-10.141,7.5}
// };

const int M = 4;
matrix::Matrix3f Qf = matrix::eye<float, 3>() * 200;
matrix::Matrix3d Identity = matrix::eye<double, 3>();


float pitch_sp_trans_f(float t, float trans_duration){	// returns pitch_sp(time) for forward transition

	float dt = trans_duration/35;
	dt = 0.1;
	double time = (double) t;
	int index = static_cast<int>(t / dt);
	double pitch_des;
	if (index < 0){
		return -1;
	} else if (index > 35){
		pitch_des = pitch_traj[35];
	} else {
		double t_low = index * dt;
		double t_high = (index + 1)*dt;
		pitch_des = pitch_traj[index] + ((pitch_traj[index+1] - pitch_traj[index]) / (t_high - t_low)) * (time - t_low);
	}
	return pitch_des;

}

std::vector<double> NDI_cont(double time, double trans_duration, float pos_0[3], double yaw){

	_vehicle_attitude_sub.update(&_v_att);
	_local_pos_sub.update(&_local_pos);

	double vx =  _local_pos.vx;
	double vy =  _local_pos.vy;
	double vz =  _local_pos.vz;
	matrix::Quatf quatern(_v_att.q);
	const matrix::Vector3f pos(0,_local_pos.y - pos_0[1],_local_pos.z-pos_0[2]);
	const matrix::Vector3f velI(vx,vy,vz);
	double pitch = matrix::Eulerf(quatern).theta();
	// double yaw = matrix::Eulerf(quatern).psi();
	matrix::Quatf q_heading = matrix::Eulerf(0.0f, 0.0f, yaw);

	double dt = trans_duration/35;
	dt = 0.1;
	int index = static_cast<int>(time/dt);

	// define setpoint velocities and accelerationa at current 'time'
	double y_sp, z_sp;
	double vy_sp, vz_sp;
	double ay_sp, az_sp;
	if (index < 0){
		y_sp = 0;
		z_sp = 0;
		vy_sp = 0;
		vz_sp = 0;
		ay_sp = 0;
		az_sp = 0;
	} else if (index > 34){
		y_sp = _local_pos.y - pos_0[1];
		z_sp = ref_pos[2][35];
		vy_sp = 14;
		vz_sp = 0;
		ay_sp = 0;
		az_sp = 0;
	} else {
		double t_low = index * dt;
		double t_high = (index + 1)*dt;
		y_sp = ref_pos[1][index] + ((ref_pos[1][index+1] - ref_pos[1][index])/(t_high - t_low)) * (time - t_low);
		z_sp = ref_pos[2][index] + ((ref_pos[2][index+1] - ref_pos[2][index])/(t_high - t_low)) * (time - t_low);
		vy_sp = ref_vel[1][index] + ((ref_vel[1][index+1] - ref_vel[1][index])/(t_high - t_low)) * (time - t_low);
		vz_sp = ref_vel[2][index] + ((ref_vel[2][index+1] - ref_vel[2][index])/(t_high - t_low)) * (time - t_low);
		// ay_sp = ((ref_vel[1][index+1] - ref_vel[1][index])/(t_high - t_low));
		// az_sp = ((ref_vel[2][index+1] - ref_vel[2][index])/(t_high - t_low));
		ay_sp = d_ref_vel[1][index] + ((d_ref_vel[1][index+1] - d_ref_vel[1][index])/(t_high - t_low)) * (time - t_low);
		az_sp = d_ref_vel[2][index] + ((d_ref_vel[2][index+1] - d_ref_vel[2][index])/(t_high - t_low)) * (time - t_low);
	}

	matrix::Vector3f pos_sp(0,y_sp,z_sp);
	matrix::Vector3f vel_sp(0,vy_sp,vz_sp);
	matrix::Vector3f acc_sp(0,ay_sp,az_sp);
	matrix::Vector3f velB = quatern.rotateVectorInverse(velI);
	matrix::Vector3f velB_sp = quatern.rotateVectorInverse(vel_sp);
	(void) velB;
	(void)velB_sp;

	matrix::Vector3f vel_sp_heading(vy_sp,0,vz_sp);
	matrix::Vector3f vel_sp_iner = q_heading.rotateVector(vel_sp_heading);
	matrix::Vector3f acc_sp_heading(ay_sp,0,az_sp);
	matrix::Vector3f acc_sp_iner = q_heading.rotateVector(acc_sp_heading);
	velB_sp = quatern.rotateVectorInverse(vel_sp_iner);
	// std::cout << time << "," << yaw*180/M_PI << "," << vel_sp(0) << "," <<  vel_sp(1) << "," <<  vel_sp(2) << "," <<  vel_sp_iner(0) << "," << vel_sp_iner(1) << "," << vel_sp_iner(2) << std::endl;

	// double e_vy = (vy_sp - vy);
	// double e_vz = (vz_sp - vz);
	matrix::Vector3f gains(1.0f,1.0f,1.0f);
	matrix::Matrix3f Gain = matrix::diag(gains);
	(void) Gain;
	// matrix::Vector3f virtualC = 0.0f * Gain * (pos_sp-pos) +  Gain * (vel_sp-velI);		// Virtual control input
	// matrix::Vector3f virtualC = Gain/2 * (pos_sp-pos);		// Virtual control input
	matrix::Vector3f virtualC = Gain * (vel_sp_iner-velI);		// Virtual control input
	matrix::Vector3f virtualC2 = Gain*(velB_sp-velB);		// Virtual control input
	matrix::Vector3f virtualC3 = quatern.rotateVector(virtualC2);
	(void) virtualC;
	(void) virtualC3;

	double gamma = atan2(-vz,vy);
	double alpha = M_PI_2 - (-pitch) - gamma;
	float speed = sqrt(velI*velI);
	matrix::Vector3f aeroF = aerodyn_forces(pitch, alpha, speed);
	matrix::Vector3f aeroF_I = quatern.rotateVector(aeroF);
	matrix::Vector3f acc = (velI-vel_prev)/(time-t_prev);
	(void) acc;
	eV_int = eV_int +  (vel_sp_iner-velI) * (time-t_prev);
	(void) eV_int;
	// virtualC = virtualC + ( Gain / 1 ) * eV_int;

	matrix::Vector3f Fg(0,0,m*9.81);
	if (time > 6){
		virtualC = virtualC/50;
		// std::cout << "Reduced Gains" << std::endl;
	}
	matrix::Vector3f DesT = virtualC*2.72 - aeroF_I - Fg + acc_sp_iner*2.72;


	matrix::Vector3f T_body = quatern.rotateVectorInverse(DesT);

	double delTheta = atan2(T_body(0),-T_body(2));
	// delTheta = atan2(DesT(1),-DesT(2));	// Actual desired thete wrt desired direction of rotor thrust

	double thrust0 = sqrt(DesT*DesT)/4;
	double scalar = 1 - (-velB(2)/25);
	double thrust = thrust0/scalar;
	double mC = 2.7e-5;
	double rpm = sqrt(thrust/mC);
	// std::cout << "Calculated Thrust and motor velocity is: " << thrust0 << "," << rpm << std::endl;
	double input = rpm/1000;
	t_prev = time;
	vel_prev = velI;
	// if (input > 0.8){
	// 	input = 0.8;
	// 	std::cout << "Zor lagaa ke Haishaaa" << std::endl;
	// }
	// if (time > 3.5) {
	// 	delTheta = 83*M_PI/180;
	// }
	// std::cout << thrust << " : " << rpm << " : " << input << std::endl;
	std::vector<double> result({-input,delTheta});
	// std::cout << "Time: " << time << ",";
	// std::cout << time << "," << aeroF_I(0) << "," << aeroF_I(1) << "," << aeroF_I(2) << "," << acc_sp(0) << "," << acc_sp(1) << "," << acc_sp(2) << ","  << DesT(0) << "," << DesT(1) << "," << DesT(2) << ","
	// 	<< vel_sp(0) << "," << vel_sp(1) << "," << vel_sp(2) << "," << velI(0) << "," << velI(1) << "," << velI(2) << "," << alpha*180/M_PI << "," << pitch*180/M_PI << std::endl;
	// std::cout << time << "," << velB_sp(0) << "," << velB_sp(1) << "," << velB_sp(2) << "," << velB(0) << "," << velB(1) << "," << velB(2) << ","
	// 	<< vel_sp_iner(0) << "," << vel_sp_iner(1) << "," << vel_sp_iner(2) << "," << velI(0) << "," << velI(1) << "," << velI(2) << "," << delTheta*180/M_PI << "," << pitch*180/M_PI << ","
	// 	<< pos_sp(0) << "," << pos_sp(1) << "," << pos_sp(2) << "," << pos(0) << "," << pos(1) << "," << pos(2);
	// std::cout << time << eV_int(0) << ":" << eV_int(1) << ":" << eV_int(2);
	return result;
}



std::vector<double> OptimalControl(float time,  float pos_0[3]){

	_vehicle_attitude_sub.update(&_v_att);
	_local_pos_sub.update(&_local_pos);

	float time_f = 3.5f;
	float dt = time_f/15;
	int index = static_cast<int>(time/dt);

	// define setpoint velocities and accelerationa at current 'time'
	float z_sp, vy_sp, vz_sp;
	float lam0, lam1, lam2;
	float T_sp, Theta_sp;


	float t_low = index * dt;
	float t_high = (index + 1) * dt;
	z_sp = state[0][index] + ( (state[0][index+1] - state[0][index]) / (t_high - t_low) ) * (time - t_low);
	vy_sp = state[1][index] + ( (state[1][index+1] - state[1][index]) / (t_high - t_low) ) * (time - t_low);
	vz_sp = state[2][index] + ( (state[2][index+1] - state[2][index]) / (t_high - t_low) ) * (time - t_low);
	lam0 = costate[0][index] + ( (costate[0][index+1] - costate[0][index]) / (t_high - t_low) ) * (time - t_low);
	lam1 = costate[1][index] + ( (costate[1][index+1] - costate[1][index]) / (t_high - t_low) ) * (time - t_low);
	lam2 = costate[2][index] + ( (costate[2][index+1] - costate[2][index]) / (t_high - t_low) ) * (time - t_low);
	T_sp = control[0][index] + ( (control[0][index+1] - control[0][index]) / (t_high - t_low) ) * (time - t_low);
	Theta_sp = control[1][index] + ( (control[1][index+1] - control[1][index]) / (t_high - t_low) ) * (time - t_low);

	float z = _local_pos.z - pos_0[2];
	float vy = _local_pos.vy;
	float vz = _local_pos.vz;
	matrix::Quatf quatern(_v_att.q);
	float pitch = matrix::Eulerf(quatern).theta();
	const matrix::Vector3f velI(0,vy,vz);
	matrix::Vector3f velB = quatern.rotateVectorInverse(velI);

	matrix::Vector3f desX(z_sp,vy_sp,vz_sp);
	matrix::Vector3f currX(z,vy,vz);
	matrix::Vector3f des_lamd(lam0,lam1,lam2);
	matrix::Vector2f desU(T_sp,Theta_sp);

	matrix::Matrix<double, 3*M, 3*M> matP;
	matrix::Matrix<double, 3*M, 3*M> matQ;
	matrix::Matrix<double, 3*M, 3*M> matR;
	matrix::Matrix<double, 3*M, 3*M> matS;
	matrix::Matrix<double, 3, 3*M> matPbar;
	matrix::Matrix<double, 3, 3*M> matQbar;


	if (index < 0){
		T_sp = control[0][0];
		Theta_sp = control[1][0];
	} else if (index > 14){
		T_sp = 2.72*9.81;
		Theta_sp = control[1][15];
	} else {
		for (int j = 0; j < M; j++) {
			float t_lgl = 0.5f * ( (time_f - time) * Tau[j] + (time_f + time) );

			int indx = static_cast<int>(t_lgl/dt);

			if (indx > 14) {
				z_sp = state[0][15];
				vy_sp = state[1][15];
				vz_sp = state[2][15];
				lam0 = costate[0][15];
				lam1 = costate[1][15];
				lam2 = costate[2][15];
				T_sp = control[0][15];
				Theta_sp = control[1][15];
			} else {
				t_low = indx * dt;
				t_high = (indx + 1) * dt;

				z_sp = state[0][indx] + ( (state[0][indx+1] - state[0][indx]) / (t_high - t_low) ) * (t_lgl - t_low);
				vy_sp = state[1][indx] + ( (state[1][indx+1] - state[1][indx]) / (t_high - t_low) ) * (t_lgl - t_low);
				vz_sp = state[2][indx] + ( (state[2][indx+1] - state[2][indx]) / (t_high - t_low) ) * (t_lgl - t_low);
				lam0 = costate[0][indx] + ( (costate[0][indx+1] - costate[0][indx]) / (t_high - t_low) ) * (t_lgl - t_low);
				lam1 = costate[1][indx] + ( (costate[1][indx+1] - costate[1][indx]) / (t_high - t_low) ) * (t_lgl - t_low);
				lam2 = costate[2][indx] + ( (costate[2][indx+1] - costate[2][indx]) / (t_high - t_low) ) * (t_lgl - t_low);
				T_sp = control[0][indx] + ( (control[0][indx+1] - control[0][indx]) / (t_high - t_low) ) * (t_lgl - t_low);
				Theta_sp = control[1][indx] + ( (control[1][indx+1] - control[1][indx]) / (t_high - t_low) ) * (t_lgl - t_low);

			}

			matrix::Vector3f X(z_sp,vy_sp,vz_sp);
			matrix::Vector3f lamd(lam0,lam1,lam2);
			matrix::Vector2f U(T_sp,Theta_sp);


			// calculate gradient of f wrt x
			matrix::Matrix<double, 3, 3> f_x;
			for (int a=0; a<3; a++) {
				matrix::Vector3f Xper = X;
				Xper(a) = X(a) + pert;
				matrix::Vector3d dXper = dynamics2(Xper, U);
				matrix::Vector3d dX = dynamics2(X, U);

				for (int i=0; i<3; i++) {
					f_x(i,a) = ( dXper(i) - dX(i) ) / pert2;
				}
			}

			// calculate gradient of f wrt u
			matrix::Matrix<double, 3, 2> f_u;
			for (int a=0; a<2; a++) {
				matrix::Vector2f Uper = U;
				Uper(a) = U(a) + pert;
				matrix::Vector3d dXper = dynamics2(X, Uper);
				matrix::Vector3d dX = dynamics2(X, U);

				for (int i=0; i<3; i++) {
					f_u(i,a) = ( dXper(i) - dX(i) ) / pert2;
				}
			}

			// Calculation of Hessian of Hamilton
			matrix::Matrix<double, 2, 3> Hux;
			for (int a=0; a<2; a++) {
				matrix::Vector2f Uper = U;
				Uper(a) = U(a) + pert;

				for (int b=0; b<3; b++) {
					matrix::Vector3f Xper = X;
					Xper(b) = X(b) + pert;

					Hux(a,b) = ( hamilton2(Xper, Uper, lamd)
							- hamilton2(X, Uper, lamd)
							- hamilton2(Xper, U, lamd)
							+ hamilton2(X, U, lamd) ) / (pert2 * pert2);
				}
			}

			matrix::Matrix<double, 3, 3> Hxx;
			for (int a=0; a<3; a++) {
				matrix::Vector3f X1 = X;
				matrix::Vector3f X0 = X;
				X1(a) = X(a) + pert;

				for (int b=0; b<3; b++) {
					matrix::Vector3f X11 = X1;
					matrix::Vector3f X10 = X1;
					matrix::Vector3f X01 = X0;
					matrix::Vector3f X00 = X0;

					X11(b) = X1(b) + pert;
					X01(b) = X0(b) + pert;

					Hxx(a,b) = (hamilton2(X11, U, lamd)
							- hamilton2(X10, U, lamd)
							- hamilton2(X01, U, lamd)
							+ hamilton2(X00, U, lamd)) / (pert2 * pert2);
				}
			}

			matrix::Matrix<double, 2, 2> Huu;
			for (int a=0; a<2; a++) {
				matrix::Vector2f U1 = U;
				matrix::Vector2f U0 = U;
				U1(a) = U(a) + pert;

				for (int b=0; b<2; b++) {
					matrix::Vector2f U11 = U1;
					matrix::Vector2f U10 = U1;
					matrix::Vector2f U01 = U0;
					matrix::Vector2f U00 = U0;

					U11(b) = U1(b) + pert;
					U01(b) = U0(b) + pert;

					Huu(a,b) = (hamilton2(X, U11, lamd)
							- hamilton2(X, U10, lamd)
							- hamilton2(X, U01, lamd)
							+ hamilton2(X, U00, lamd)) / (pert2 * pert2);
				}
			}

			bool time1 = (-0.0001f < time) && (time < 0.0001f);
			bool time2 = (0.235f < time) && (time < 0.237f);
			bool time3 = (1.399f < time) && (time < 1.41f);
			if (time1 || time2 || time3) {
				// std::cout << "X: " << X(0) << " " << X(1) << " " << X(2) << std::endl;
				// std::cout << "lamd: " << lamd(0) << " " << lamd(1) << " " << lamd(2) << std::endl;
				// std::cout << "U: " << U(0) << " " << U(1) << " " << U(2) << std::endl;

			// 	std::cout << "fx:" << std::endl;
			// 	for (int a=0; a<3; a++) {
			// 		for (int b=0; b<3; b++) {
			// 			std::cout << f_x(a,b) << " ";
			// 		}
			// 		std::cout << std::endl;
			// 	}
			// 	std::cout << "fu:" << std::endl;
			// 	for (int a=0; a<3; a++) {
			// 		for (int b=0; b<2; b++) {
			// 			std::cout << f_u(a,b) << " ";
			// 		}
			// 		std::cout << std::endl;
			// 	}
			// 	std::cout << "Hux:" << std::endl;
			// 	for (int a=0; a<2; a++) {
			// 		for (int b=0; b<3; b++) {
			// 			std::cout << Hux(a,b) << " ";
			// 		}
			// 		std::cout << std::endl;
			// 	}
			// 	std::cout << "Hxx:" << std::endl;
			// 	for (int a=0; a<3; a++) {
			// 		for (int b=0; b<3; b++) {
			// 			std::cout << Hxx(a,b) << " ";
			// 		}
			// 		std::cout << std::endl;
			// 	}
			// 	std::cout << "Huu:" << std::endl;
			// 	for (int a=0; a<2; a++) {
			// 		for (int b=0; b<2; b++) {
			// 			std::cout << Huu(a,b) << " ";
			// 		}
			// 		std::cout << std::endl;
			// 	}
			}


			Eigen::MatrixXd testMat(2, 2);
			testMat(0,0) = Huu(0,0);
			testMat(0,1) = Huu(0,1);
			testMat(1,0) = Huu(1,0);
			testMat(1,1) = Huu(1,1);

			// Compute the pseudo-inverse of the identity matrix
			Eigen::MatrixXd pseudoInv = pseudoInverse(testMat);

			matrix::Matrix<double, 2, 2> invHuu;
			matrix::Matrix<double, 3, 3> A;
			matrix::Matrix<double, 3, 3> B;
			matrix::Matrix<double, 3, 3> C;

			invHuu(0,0) = pseudoInv(0,0);
			invHuu(0,1) = pseudoInv(0,1);
			invHuu(1,0) = pseudoInv(1,0);
			invHuu(1,1) = pseudoInv(1,1);

			A = f_x - f_u * (invHuu * Hux);
			B = - f_u * (invHuu * f_u.transpose());
			C = Hxx - Hux.transpose() * (invHuu * Hux);

		/****************************************************************************** */

			// assign values to matP, matQ, matR, matS, matPbar, matQbar
			for (int k=0; k<M; k++) {

				double D_jk = D_mat[j][k];

				if (k == j) {
					for (int a=0; a<3; a++) {
						for (int b=0; b<3; b++) {
							matP(3*j+a,3*k+b) = D_jk * Identity(a,b) - 0.5 * (static_cast<double>(time_f) - static_cast<double>(time)) * A(a,b);
							matQ(3*j+a,3*k+b) = 0.5 * (static_cast<double>(time_f) - static_cast<double>(time)) * B(a,b);
							matR(3*j+a,3*k+b) = 0.5 * (static_cast<double>(time_f) - static_cast<double>(time)) * C(a,b);
							matS(3*j+a,3*k+b) = D_jk * Identity(a,b) + 0.5 * (static_cast<double>(time_f) - static_cast<double>(time)) * A(b,a);
						}
					}
				} else {
					for (int a=0; a<3; a++) {
						for (int b=0; b<3; b++) {
							matP(3*j+a,3*k+b) = D_jk * Identity(a,b);
							matS(3*j+a,3*k+b) = D_jk * Identity(a,b);
						}
					}
				}

				if (k == M-1) {
					for (int a=0; a<3; a++) {
						matPbar(a,3*k+a) = 2 * Qf(a,a);
						matQbar(a,3*k+a) = -1;
					}
				}
			}
		}

		// the matrices P,Q,R,S,Pbar,Qbar are generated
		// Now create the V matrix and perform linear algebra

		Eigen::MatrixXd V0(6*M+3, 3);
		Eigen::MatrixXd Ve(6*M+3, 6*M-3);
		Eigen::MatrixXd Z0(3, 1);
		Eigen::MatrixXd Ze(6*M-3, 1);

		for (int a=0; a<3*M; a++) {

			// V0 <= P, V0 <-= R
			for (int b=0; b<3; b++) {
				V0(a,b) = matP(a,b);
				V0(3*M+a,b) = matR(a,b);
			}

			// Ve <= P, Ve <= R
			for (int b=0; b<3*M-3; b++) {
				Ve(a,b) = matP(a,3+b);
				Ve(3*M+a,b) = matR(a,3+b);
			}

			// Ve <= Q, Ve <= S
			for (int b=0; b<3*M; b++) {
				Ve(a,3*M-3+b) = matQ(a,b);
				Ve(3*M+a,3*M-3+b) = matS(a,b);
			}
		}

		for (int a=0; a<3; a++) {

			// V0 <= Pbar
			for (int b=0; b<3; b++) {
				V0(6*M+a,b) = matPbar(a,b);
			}

			// Ve <= Pbar
			for (int b=0; b<3*M-3; b++) {
				Ve(6*M+a,b) = matPbar(a,3+b);
			}

			// Ve <= Qbar
			for (int b=0; b<3*M; b++) {
				Ve(6*M+a,3*M-3+b) = matQbar(a,b);
			}
		}

		// Compute the pseudo-inverse of the identity matrix
		Eigen::MatrixXd Inv_Ve = pseudoInverse(Ve);


		bool time1 = (-0.0001f < time) && (time < 0.0001f);
		bool time2 = (0.231f < time) && (time < 0.237f);
		bool time3 = (1.39f < time) && (time < 1.41f);
		if (time1 || time2 || time3) {
			// std::cout << "V:" << std::endl;
			// for (int row = 0; row<(6*M)+3; row++) {
			// 	for (int column=0; column<6*M; column++) {
			// 		std::cout << V(row,column) << " ";
			// 	}
			// 	std::cout << std::endl;
			// }
			// std::cout << "V0:" << std::endl;
			// for (int row = 0; row<(6*M+3); row++) {
			// 	for (int column=0; column<3; column++) {
			// 		std::cout << V0(row,column) << " ";
			// 	}
			// 	std::cout << std::endl;
			// }
			// std::cout << "invVe:" << std::endl;
			// for (int row = 0; row<(6*M)-3; row++) {
			// 	for (int column=0; column<6*M+3; column++) {
			// 		std::cout << Inv_Ve(row,column) << " ";
			// 	}
			// 	std::cout << std::endl;
			// }
		}

		matrix::Vector3f temp_delX = desX - currX;
		matrix::Vector3d delX;
		delX(0) = static_cast<double> (temp_delX(0));
		delX(1) = static_cast<double> (temp_delX(1));
		delX(2) = static_cast<double> (temp_delX(2));

		Z0(0,0) = delX(0);
		Z0(1,0) = delX(1);
		Z0(2,0) = delX(2);

		Ze = - Inv_Ve * (V0 * Z0);

		matrix::Vector3d del_Lmbd;
		del_Lmbd(0) = Ze(3*M-3,0);
		del_Lmbd(1) = Ze(3*M-2,0);
		del_Lmbd(2) = Ze(3*M-1,0);

		// calculate gradient of f wrt u
		matrix::Matrix<double, 3, 2> f_u;
		for (int a=0; a<2; a++) {
			matrix::Vector2f desUper = desU;
			desUper(a) = desU(a) + pert;
			matrix::Vector3d dXper = dynamics2(desX, desUper);
			matrix::Vector3d dX = dynamics2(desX, desU);

			for (int i=0; i<3; i++) {
				f_u(i,a) = ( dXper(i) - dX(i) ) / pert2;
			}
		}

		// Calculation of Hessian of Hamilton
		matrix::Matrix<double, 2, 3> Hux;
		for (int a=0; a<2; a++) {
			matrix::Vector2f desUper = desU;
			desUper(a) = desU(a) + pert;

			for (int b=0; b<3; b++) {
				matrix::Vector3f desXper = desX;
				desXper(b) = desX(b) + pert;

				Hux(a,b) = ( hamilton2(desXper, desUper, des_lamd)
						- hamilton2(desX, desUper, des_lamd)
						- hamilton2(desXper, desU, des_lamd)
						+ hamilton2(desX, desU, des_lamd) ) / (pert2 * pert2);
			}
		}

		matrix::Matrix<double, 2, 2> Huu;
		for (int a=0; a<2; a++) {
			matrix::Vector2f desU1 = desU;
			matrix::Vector2f desU0 = desU;
			desU1(a) = desU(a) + pert;

			for (int b=0; b<2; b++) {
				matrix::Vector2f desU11 = desU1;
				matrix::Vector2f desU10 = desU1;
				matrix::Vector2f desU01 = desU0;
				matrix::Vector2f desU00 = desU0;

				desU11(b) = desU1(b) + pert;
				desU01(b) = desU0(b) + pert;

				Huu(a,b) = (hamilton2(desX, desU11, des_lamd)
						- hamilton2(desX, desU10, des_lamd)
						- hamilton2(desX, desU01, des_lamd)
						+ hamilton2(desX, desU00, des_lamd)) / (pert2 * pert2);
			}
		}
		Eigen::MatrixXd Huu_temp(2, 2);
		Huu_temp(0,0) = Huu(0,0);
		Huu_temp(0,1) = Huu(0,1);
		Huu_temp(1,0) = Huu(1,0);
		Huu_temp(1,1) = Huu(1,1);

		// Compute the pseudo-inverse of the identity matrix
		Eigen::MatrixXd invHuu_temp = pseudoInverse(Huu_temp);

		matrix::Matrix<double, 2, 2> invHuu;
		invHuu(0,0) = invHuu_temp(0,0);
		invHuu(0,1) = invHuu_temp(0,1);
		invHuu(1,0) = invHuu_temp(1,0);
		invHuu(1,1) = invHuu_temp(1,1);

		matrix::Vector2d delU = - invHuu * ( Hux * delX + f_u.transpose() * del_Lmbd);

		matrix::Vector2f newU;
		newU(0) = desU(0) + static_cast<float>(delU(0));
		newU(1) = desU(1) + static_cast<float>(delU(1));


		/****************************************************************************** */
		T_sp = newU(0);
		Theta_sp = newU(1);

		std::cout << time << " " << _local_pos.y << " " << desX(0) << " " << desX(1) << " " << desX(2) << " " << currX(0) << " " << currX(1) << " " << currX(2)
		 << " " << Theta_sp << " " << pitch;

	}

	// normalise T_sp to (0,1)

	float scalar = 1 - (-velB(2)/25);
	float thrust = (T_sp/4)/scalar;
	float mC = 2.7e-5;
	float rpm = sqrt(thrust/mC);
	// std::cout << "Calculated Thrust and motor velocity is: " << thrust0 << "," << rpm << std::endl;
	double input = rpm/1000;


	std::vector<double> result({-input,Theta_sp});
	(void) pitch;
	// std::cout << "Code is Running" << std::endl;
	return result;


}


matrix::Vector3f aerodyn_forces(double theta, double alpha, double V){
	// #include "params.hpp"

	// dynamic pressure
	double q = 0.5 * rho * V * V;

	// calculate lift and drag
	double sigma = ( 1 + exp(-stall_rate*(alpha-alpha_0)) + exp(stall_rate*(alpha+alpha_0)) )
	 		/ ( (1 + exp(-stall_rate*(alpha-alpha_0))) * (1 + exp(stall_rate*(alpha+alpha_0))) );
	double CL = ( 1 - sigma ) * ( CL0 + CLa * alpha )
			+ sigma * ( 0.8 * (alpha/fabs(alpha)) * sin(alpha) * sin(alpha) * cos(alpha) );
	double CD =  CD0 + (1/(M_PI/0.22)) * CL * CL + sin(alpha) * sin(alpha);

	double lift = q * 2*area * CL;
	double drag = q * 2*area * CD;
	double Fx = - lift * cos(alpha) - drag * sin(alpha);
	double Fz = -lift * sin(alpha) + drag * cos(alpha);

	// std::cout << alpha << "," << CL << "," << CD << "," << q << "," << Fx/2 << "," << Fz/2 << std::endl;
	matrix::Vector3f NetForce(Fx, 0, Fz);
	return NetForce;
}

matrix::Vector3f dynamics(matrix::Vector3f X, matrix::Vector2f Control){
	float vy = X(1);
	float vz = X(2);
	float Thr = Control(0);
	float theta = Control(1);

	float gamma = atan2(-vz,vy);
	float alpha = M_PI_2_F - theta - gamma;
	float speed = sqrt(vy*vy + vz*vz);
	matrix::Vector3f aeroF = aerodyn_forces(theta, alpha, speed);

	matrix::Vector3f xdot;
	xdot(0) = vz;
	xdot(1) = ( Thr*sin(theta) + aeroF(0)*cos(theta) - aeroF(2)*sin(theta) ) / 2.72f;
	xdot(2) = ( -Thr*cos(theta) + aeroF(0)*sin(theta) + aeroF(2)*cos(theta) ) / 2.72f + 9.81f;

	return xdot;
}

float hamilton(matrix::Vector3f X, matrix::Vector2f Control, matrix::Vector3f lambda) {
	float Thr = Control(0);
	float theta = Control(1);
	float J =  Thr - 2.72f * 9.81f * cos(theta);
	matrix::Matrix<float, 1, 1> lam_dyn = lambda.transpose()*dynamics(X,Control);
	float H = J * J + lam_dyn(0,0);

	return H;
}


matrix::Vector3d aerodyn_forces2(double theta, double alpha, double V){
	// #include "params.hpp"

	// dynamic pressure
	double q = 0.5 * rho * V * V;

	// calculate lift and drag
	double sigma = ( 1 + exp(-stall_rate*(alpha-alpha_0)) + exp(stall_rate*(alpha+alpha_0)) )
	 		/ ( (1 + exp(-stall_rate*(alpha-alpha_0))) * (1 + exp(stall_rate*(alpha+alpha_0))) );
	double CL = ( 1 - sigma ) * ( CL0 + CLa * alpha )
			+ sigma * ( 0.8 * (alpha/fabs(alpha)) * sin(alpha) * sin(alpha) * cos(alpha) );
	double CD =  CD0 + (1/(M_PI/0.22)) * CL * CL + sin(alpha) * sin(alpha);

	double lift = q * 2*area * CL;
	double drag = q * 2*area * CD;
	double Fx = - lift * cos(alpha) - drag * sin(alpha);
	double Fz = -lift * sin(alpha) + drag * cos(alpha);

	// std::cout << alpha << "," << CL << "," << CD << "," << q << "," << Fx/2 << "," << Fz/2 << std::endl;
	matrix::Vector3d NetForce(Fx, 0, Fz);
	return NetForce;
}

double hamilton2(matrix::Vector3f X, matrix::Vector2f Control, matrix::Vector3f lambda) {
	double Thr = Control(0);
	double theta = Control(1);
	double J =  Thr - 2.72 * 9.81 * cos(theta);
	matrix::Matrix<float, 1, 3> lam_Tf = lambda.transpose();
	matrix::Matrix<double, 1, 3> lam_T;
	lam_T(0,0) = (double) lam_Tf(0,0);
	lam_T(0,1) = (double) lam_Tf(0,1);
	lam_T(0,2) = (double) lam_Tf(0,2);
	matrix::Matrix<double, 1, 1> lam_dyn = lam_T*dynamics2(X,Control);
	double H = J * J + lam_dyn(0,0);

	return H;
}


matrix::Vector3d dynamics2(matrix::Vector3f X, matrix::Vector2f Control){
	double vy = X(1);
	double vz = X(2);
	double Thr = Control(0);
	double theta = Control(1);

	double gamma = atan2(-vz,vy);
	double alpha = M_PI_2 - theta - gamma;
	double speed = sqrt(vy*vy + vz*vz);
	matrix::Vector3d aeroF = aerodyn_forces2(theta, alpha, speed);

	matrix::Vector3d xdot;
	xdot(0) = vz;
	xdot(1) = ( Thr*sin(theta) + aeroF(0)*cos(theta) - aeroF(2)*sin(theta) ) / 2.72;
	xdot(2) = ( -Thr*cos(theta) + aeroF(0)*sin(theta) + aeroF(2)*cos(theta) ) / 2.72 + 9.81;

	return xdot;
}
