/****************************************************************************
 * Date: May 9, 2024
 * Author: Shubhanshu Gupta
 *
 * A generated trajecotry for transition of tailsitter vehicle
****************************************************************************/

#include <math.h>
#include <iostream>
#include "../../mc_pos_control/MulticopterPositionControl.hpp"
#include "../../mc_pos_control/PositionControl/PositionControl.hpp"
#include <matrix/math.hpp>

#include <uORB/Subscription.hpp>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_local_position.h>


uORB::Subscription	_vehicle_attitude_sub{ORB_ID(vehicle_attitude)};
uORB::Subscription 	_local_pos_sub{ORB_ID(vehicle_local_position)};			// sensor subscription

vehicle_attitude_s		_v_att{};
vehicle_local_position_s	_local_pos{};

float t_off = 0;


double pitch_traj[36] = {0,0.0469405273861218,0.145794459164360,0.238529668683919,0.308065396591440,0.357824714834710,0.397507802823208,0.434537004859852,0.473208665495969,0.517647465708863,0.571694339663055,0.636942159439146,0.711636202250065,0.792162539861533,0.874423054978880,0.954721744121555,1.03006917088842,1.09853392771779,1.15942302912874,1.21353666413073,1.26286703359881,1.30919768431893,1.35306855641429,1.39343507055640,1.42827618212329,1.45606528713739,1.47634767259439,1.48924669622426,1.49478027740913,1.49262841761920,1.48229384738287,1.46371340992890,1.43997038149566,1.42058811921648,1.41862751271676,1.41714609262828};

double ref_pos[3][36] = {
	{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
	{0,	0.000498464116464567,	0.00750222901197887,	0.0327262954583019,	0.0869243352284179,	0.177626105440049,	0.309811106061309,	0.486952438161038,	0.711666355875733,	0.986121682923955,	1.31233887703359,	1.69237082707463,	2.12812636483339,	2.62080493521087,	3.17036913863222,	3.77534559206481,	4.43300826557965,	5.13986723215190,	5.89230542760286,	6.68705020908791,	7.52141134917359,	8.39329912001758,	9.30118945475915,	10.2440830827550,	11.2214410986929,	12.2332455384086,	13.2800136700903,	14.3626495503572,	15.4821756511723,	16.6394801871173,	17.8352052412536,	19.0697590478179,	20.3433284065544,	21.6556198326141,	23.0051669788527,	24.3889507994378},
	{0,	0.0244771104629748,	0.0778224221273357,	0.158805462365442,	0.264766886625530,	0.392836175489671,	0.540465877141363,	0.705032709729423,	0.883572775983538,	1.07276776856693,	1.26869626160705,	1.46652722516086,	1.66033001285510,	1.84318894215463,	2.00776785067014,	2.14709512940166,	2.25525617057379,	2.32783372948019,	2.36206728863351,	2.35674147263019,	2.31197651342433,	2.22908689359641,	2.11057443275819,	1.96020443058209,	1.78298661068561,	1.58499162533508,	1.37304383344061,	1.15440343243462,	0.936473615769387,	0.726553821367629,	0.531759710350203,	0.359219515584493,	0.216416593071030,	0.111038385408255,	0.0488605094967859,	0.0281917745053911}
};

double ref_vel[3][36] = {
	{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
	{0,	0.0198686536274475,	0.140850984930529,	0.382074558318832,	0.714736542347956,	1.10792661797153,	1.54209359138797,	2.00586828258124,	2.49300272297672,	3.00087849685533,	3.52912637369472,	4.07830386057550,	4.64428912298357,	5.21673960949790,	5.78137707468326,	6.32405227156646,	6.83409612892430,	7.30704441082292,	7.74489502521520,	8.15266624363671,	8.53718910867861,	8.90369088634921,	9.25804839667145,	9.60443833627977,	9.94744790564775,	10.2928242982863,	10.6456788800415,	11.0087624087561,	11.3817719325133,	11.7624685908320,	12.1483360612386,	12.5372593591542,	12.9269350635186,	13.3102975017435,	13.6711120363549,	13.9953353738306},
	{-0.1,	-0.389273349394808,	-0.675576596141736,	-0.940564117631386,	-1.17538720842906,	-1.38326970658396,	-1.56662711788447,	-1.72177395616204,	-1.84578584002168,	-1.93446026599410,	-1.97993115360714,	-1.97204812455925,	-1.89927626580095,	-1.75365474980425,	-1.53472820390021,	-1.25003508345116,	-0.912881129215326,	-0.539676073178514,	-0.147089709718271,	0.250514122541246,	0.640514696807454,	1.01149941291651,	1.35114393800128,	1.64680413771034,	1.88664313479091,	2.06142708156532,	2.16523419494683,	2.19521776767926,	2.15134847970518,	2.03549434384785,	1.84899092142058,	1.58995096163165,	1.25381171354262,	0.843904866371878,	0.403240097683905,	0.0250026445325353}
};

double vel_int[3] = {0,0,0};
double t_prev = 0;

extern PositionControl _control; // Declare extern if _control is defined elsewhere

float pitch_sp_trans_f(float t, float trans_duration){	// returns pitch_sp(time) for forward transition

	float dt = trans_duration/35;
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
	// std::cout << "Current Time: " << t << std::endl;
	// std::cout << "Transition Time:" << trans_duration << std::endl;
	return pitch_des;
	// std::cout << "Pos Gain: " << _control._gain_pos_p(0) << std::endl;

}

double outer_loop_cont(float x, float y, float z, float vx, double vy, double vz, float z_deriv, double pitch, float pos0[3], float t, float trans_duration) {

	// Subscribe to the topic

	// std::cout << "Check update status : " << _vehicle_attitude_sub.updated() << std::endl;
	_vehicle_attitude_sub.update(&_v_att);
	_local_pos_sub.update(&_local_pos);
	std::cout << "quaternion: " << _v_att.q[0] << " : " << _v_att.q[1] << " : " << _v_att.q[2] << " : " << _v_att.q[3] << std::endl;

	std::cout << "time1: " <<  _v_att.timestamp << std::endl;
	// std::cout << "time2: " <<  _local_pos.timestamp << std::endl;

	float timesss = (float)hrt_absolute_time();
	std::cout << "Abhi ka time: " << timesss << std::endl;

	if (t_off < 0.0001f){
		t_off = timesss;
		std::cout << "t0 updated" << std::endl;
	}

	float transT = timesss - t_off;
	std::cout << "transition time" << transT << " : " << t << std::endl;

	const matrix::Vector3f velI(_local_pos.vx,_local_pos.vy,_local_pos.vz);
	matrix::Quatf quatern(_v_att.q);
	// quatern.copyTo(_v_att.q);
	// _v_att.q.copyTo(quatern);
	std::cout << "quaternion: " << _v_att.q[0] << " : " << _v_att.q[1] << " : " << _v_att.q[2] << " : " << _v_att.q[3] << std::endl;
	matrix::Vector3f velB = quatern.rotateVectorInverse(velI);

	std::cout << "VelocityI: " << velI(0) << " : " << velI(1) << " : " << velI(2) << std::endl;

	float speed = sqrt(velB*velB);
	std::cout << "Speed: " << speed << std::endl;



	x= x-pos0[0];y=y-pos0[1];z=z-pos0[2];
	// std::cout << "Pos X: " << x << "Pos Y: " << y << "Pos Z: " << z << std::endl;
	// std::cout << "Vel X: " << vx << " Vel Y: " << vy << " Vel Z: " << vz << " derZ: " << z_deriv << std::endl;
	// std::cout << "Pos0 received" << pos0[0] << std::endl;

	float dt = trans_duration/35;
	double time = (double) t;
	int index = static_cast<int>(t / dt);
	// double bodyvel[3];
	// matrix::Vector3f inervel[3] = {vx, vy,vz};

	double vy_sp, vz_sp;
	if (index < 0){
		vy_sp = 0;
		vz_sp = 0;
	} else if (index > 35){
		vy_sp = 14;
		vz_sp = 0;
	} else {
		double t_low = index * dt;
		double t_high = (index + 1)*dt;
		vy_sp = ref_vel[1][index] + ((ref_vel[1][index+1] - ref_vel[1][index])/(t_high - t_low)) * (time - t_low);
		vz_sp = ref_vel[2][index] + ((ref_vel[2][index+1] - ref_vel[2][index])/(t_high - t_low)) * (time - t_low);
	}

	double del_t = time-t_prev;
	std::cout << "Current Time: " << t << std::endl;
	// std::cout << "Prev Time: " << t_prev << std::endl;
	// std::cout << "Diff: " << del_t << std::endl;
	vel_int[1] += (vy_sp-vy)*del_t;
	vel_int[2] += (vz_sp-vz)*del_t;

	double acc_y_sp = (vy_sp-vy)*10 + 2*vel_int[1];
	double acc_z_sp = (vz_sp-vz)*10 + 2*vel_int[2];
	// std::cout << "Current Time: " << t << std::endl;
	// std::cout << "Pitch: " << pitch*180/3.14 << std::endl;
	// std::cout << "sp_y: " << vy_sp << " sp_z: " << vz_sp << std::endl;
	// std::cout << "vely: " << vy << " velz: " << vz << std::endl;
	// std::cout << "Acc_y: " << acc_y_sp << " Acc_z: " << acc_z_sp << std::endl;

	double acc_body_z = acc_z_sp * cos(-pitch) - acc_y_sp * sin(-pitch);
	// std::cout << "Acc in body z: " << acc_body_z << std::endl;

	double net_thrust = acc_body_z*0.5/9.81 - 0.5;
	// float hover = _control.getHoverThrust();
	// std::cout << "Net Thrust: " << net_thrust << std::endl;

	// bodyvel = quat->rotateVectorInverse(inervel);


	t_prev = time;
	return net_thrust;

}







