/****************************************************************************
 * Date: Oct 27, 2024
 * Author: Shubhanshu Gupta
 *
 * Outer loop trajectory control for backward transition of tailsitter vehicle
 * using NOC control
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

/****** Already defined in NDI_controller.hpp *******
#include "params.hpp"
uORB::Subscription	_vehicle_attitude_sub{ORB_ID(vehicle_attitude)};
uORB::Subscription 	_local_pos_sub{ORB_ID(vehicle_local_position)};			// sensor subscription
vehicle_attitude_s		_v_att{};
vehicle_local_position_s	_local_pos{};
*****************************************************/

// pert and pert2 are defined in NDI_controller.hpp


// Reference trajectories for backward transition

float b_state[3][16] = {
	{-0,0.183725638998019,0.48675184401251,0.802146024547046,1.06458678867991,1.23669037313779,1.30640996821781,1.28066761049757,1.17940598009668,1.02927259799542,0.846991463262046,0.625641336102915,0.386174024854435,0.198666982803513,0.0843728393699137,0.0549077101013598},
	{15,14.542253159628,14.1719289333527,13.8708859292228,13.5941005605981,13.3022428232758,12.9673692935635,12.5746683443858,12.1216123266797,11.659325057835,11.380314014269,11.517344156666,9.42967870573899,7.37127641597255,5.62849898734638,4.11167566220493},
	{-0,1.08035685223956,1.40731568461203,1.33298235550725,1.00562826869265,0.564154566489647,0.11750974266887,-0.26803421922171,-0.543819492801233,-0.701036633194466,-0.839109152380049,-1.07889100726065,-0.947719351523428,-0.677929472549259,-0.327557337846025,0.0541935383221674}
};

float b_costate[3][16] = {
	{2.19630840405439,2.19630840405439,2.19630840405439,2.19630840405439,2.19630840405439,2.19630840405439,2.19630840405439,2.19630840405439,2.19630840405439,2.19630840405439,2.19630840405439,2.19630840405439,2.19630840405439,2.19630840405439,2.19630840405439,2.19630840405439},
	{-10.9429985297369,-11.9037502839355,-13.0004819868613,-14.1285748221804,-15.0075578258065,-15.2331237346259,-14.3246216978649,-11.7738838880397,-7.21370169558312,-1.07206042731154,3.86672366474789,8.06685277189744,12.1729596737998,15.9627373712835,19.3042334770878,22.3351324409858},
	{-0.492175632562498,-0.699834889355081,-0.48422974311485,0.370405537306761,1.99982391555143,4.46034551303311,7.74175465676081,11.7235552307354,15.7448383336649,17.1802060971065,11.435424141782,12.6662159029287,12.6290603163825,12.1641339233101,11.4519591770094,10.6387076644335}
};

float b_control[2][16] = {
	{0.917287719041199,1.21535646126215,1.75578364492984,2.45804408005284,3.16097241398738,3.63615795998077,3.78931666998032,3.59976442361473,3.71560168976519,6.28296559125332,13.1783791420573,21.7314124621077,26.1938203014934,27.0051829253298,26.8283541993545,26.8283541993545},
	{1.57244180101683,1.57455612461597,1.55423402075083,1.51639825551091,1.47070141299283,1.42479271161546,1.38187807791139,1.34454235129254,1.31223980359352,1.27525764947573,1.18564649025856,0.249483903497708,-0.126206502576265,-0.326868193871874,-0.453506501751076,-0.453506501751076}
};

float b_Time[16] = {0,0.233333333333333,0.466666666666667,0.7,0.933333333333333,1.16666666666667,1.4,1.63333333333333,1.86666666666667,2.1,2.33333333333333,2.56666666666667,2.8,3.03333333333333,3.26666666666667,3.5};

std::vector<float> b_Tau = {-1,-0.44721,0.44721,1};

using namespace matrix;

float b_D_mat[4][4] = {
	{-3,4.0451,-1.5451,0.5},
	{-0.80902,0,1.118,-0.30902},
	{0.30902,-1.118,0,0.80902},
	{-0.5,1.5451,-4.0451,3}
};

// M = 4 is alread defined in NDI_controller file
Vector3f term_weight(20,100,100);
Matrix3f b_Qf = diag(term_weight);

// Check the trajectory feasibility by trying to track the pitch trajectory
float pitch_sp_trans_b(float t, float trans_duration){

	float dt = trans_duration/15;
	int index = static_cast<int> (t/dt);
	float pitch_des;

	if (index < 0){
		pitch_des = b_control[1][0];
	} else if (index > 15){
		pitch_des = b_control[1][15];
	} else {
		float t_low = index * dt;
		float t_high = (index + 1) * dt;
		pitch_des = b_control[1][index] + ( (b_control[1][index+1] - b_control[1][index]) / (t_high - t_low) ) * (t - t_low);
	}
	return pitch_des;

}

std::vector<double> b_OptimalControl(float time,  float pos_0[3]){

	_vehicle_attitude_sub.update(&_v_att);
	_local_pos_sub.update(&_local_pos);

	float time_f = 3.5f;
	float dt = time_f/15;
	int index = static_cast<int>(time/dt);

	// define setpoint state, costate and controls at current 'time'
	float z_sp, vy_sp, vz_sp;
	float lam0, lam1, lam2;
	float T_sp, Theta_sp;

	float t_low = index * dt;
	float t_high = (index + 1) * dt;
	z_sp = b_state[0][index] + ( (b_state[0][index+1] - b_state[0][index]) / (t_high - t_low) ) * (time - t_low);
	vy_sp = b_state[1][index] + ( (b_state[1][index+1] - b_state[1][index]) / (t_high - t_low) ) * (time - t_low);
	vz_sp = b_state[2][index] + ( (b_state[2][index+1] - b_state[2][index]) / (t_high - t_low) ) * (time - t_low);
	lam0 = b_costate[0][index] + ( (b_costate[0][index+1] - b_costate[0][index]) / (t_high - t_low) ) * (time - t_low);
	lam1 = b_costate[1][index] + ( (b_costate[1][index+1] - b_costate[1][index]) / (t_high - t_low) ) * (time - t_low);
	lam2 = b_costate[2][index] + ( (b_costate[2][index+1] - b_costate[2][index]) / (t_high - t_low) ) * (time - t_low);
	T_sp = b_control[0][index] + ( (b_control[0][index+1] - b_control[0][index]) / (t_high - t_low) ) * (time - t_low);
	Theta_sp = b_control[1][index] + ( (b_control[1][index+1] - b_control[1][index]) / (t_high - t_low) ) * (time - t_low);

	float z = _local_pos.z - pos_0[2];
	float vy = _local_pos.vy;
	float vz = _local_pos.vz;
	Quatf quatern(_v_att.q);
	float pitch = Eulerf(quatern).theta();
	const Vector3f velI(0,vy,vz);  // Inertial velocity
	Vector3f velB = quatern.rotateVectorInverse(velI); // velocity in body frame

	// Define setpoint quantities
	Vector3f desX(z_sp,vy_sp,vz_sp);
	Vector3f currX(z,vy,vz);
	Vector3f des_lamd(lam0,lam1,lam2);
	Vector2f desU(T_sp,Theta_sp);

	Matrix<double, 3*M, 3*M> matP;
	Matrix<double, 3*M, 3*M> matQ;
	Matrix<double, 3*M, 3*M> matR;
	Matrix<double, 3*M, 3*M> matS;
	Matrix<double, 3, 3*M> matPbar;
	Matrix<double, 3, 3*M> matQbar;

	// Define matrices for NOC

	if (index < 0){
		T_sp = b_control[0][0];
		Theta_sp = b_control[1][0];
	} else if (index > 14){
		T_sp = 2.72*9.81;
		Theta_sp = 0;
	} else {
		for (int j = 0; j < M; j++) {
			float t_lgl = 0.5f * ( (time_f - time) * b_Tau[j] + (time_f + time) );

			int indx = static_cast<int>(t_lgl/dt);

			// failsafe for undesired values of index
			if (indx > 14){
				z_sp = b_state[0][15];
				vy_sp = b_state[1][15];
				vz_sp = b_state[2][15];
				lam0 = b_costate[0][15];
				lam1 = b_costate[1][15];
				lam2 = b_costate[2][15];
				T_sp = b_control[0][15];
				Theta_sp = b_control[1][15];
			} else {
			// find setpoint data at lgl points
				t_low = indx * dt;
				t_high = (indx + 1) * dt;

				z_sp = b_state[0][indx] + ( (b_state[0][indx+1] - b_state[0][indx]) / (t_high - t_low) ) * (t_lgl - t_low);
				vy_sp = b_state[1][indx] + ( (b_state[1][indx+1] - b_state[1][indx]) / (t_high - t_low) ) * (t_lgl - t_low);
				vz_sp = b_state[2][indx] + ( (b_state[2][indx+1] - b_state[2][indx]) / (t_high - t_low) ) * (t_lgl - t_low);
				lam0 = b_costate[0][indx] + ( (b_costate[0][indx+1] - b_costate[0][indx]) / (t_high - t_low) ) * (t_lgl - t_low);
				lam1 = b_costate[1][indx] + ( (b_costate[1][indx+1] - b_costate[1][indx]) / (t_high - t_low) ) * (t_lgl - t_low);
				lam2 = b_costate[2][indx] + ( (b_costate[2][indx+1] - b_costate[2][indx]) / (t_high - t_low) ) * (t_lgl - t_low);
				T_sp = b_control[0][indx] + ( (b_control[0][indx+1] - b_control[0][indx]) / (t_high - t_low) ) * (t_lgl - t_low);
				Theta_sp = b_control[1][indx] + ( (b_control[1][indx+1] - b_control[1][indx]) / (t_high - t_low) ) * (t_lgl - t_low);
			}

			Vector3f X(z_sp,vy_sp,vz_sp);
			Vector3f lamd(lam0,lam1,lam2);
			Vector2f U(T_sp,Theta_sp);

			// calculate gradient of f wrt x
			Matrix<double, 3, 3> f_x;
			for (int a=0; a<3; a++) {
				Vector3f Xper = X;
				Xper(a) = X(a) + pert;
				Vector3d dXper = dynamics2(Xper, U);
				Vector3d dX = dynamics2(X, U);

				for (int i=0; i<3; i++) {
					f_x(i,a) = ( dXper(i) - dX(i) ) / pert2;
				}
			}

			// calculate gradient of f wrt u
			Matrix<double, 3, 2> f_u;
			for (int a=0; a<2; a++) {
				Vector2f Uper = U;
				Uper(a) = U(a) + pert;
				Vector3d dXper = dynamics2(X, Uper);
				Vector3d dX = dynamics2(X, U);

				for (int i=0; i<3; i++) {
					f_u(i,a) = ( dXper(i) - dX(i) ) / pert2;
				}
			}

			// Calculation of Hessian of Hamilton
			Matrix<double, 2, 3> Hux;
			for (int a=0; a<2; a++) {
				Vector2f Uper = U;
				Uper(a) = U(a) + pert;

				for (int b=0; b<3; b++) {
					Vector3f Xper = X;
					Xper(b) = X(b) + pert;

					Hux(a,b) = ( hamilton2(Xper, Uper, lamd)
							- hamilton2(X, Uper, lamd)
							- hamilton2(Xper, U, lamd)
							+ hamilton2(X, U, lamd) ) / (pert2 * pert2);
				}
			}

			Matrix<double, 3, 3> Hxx;
			for (int a=0; a<3; a++) {
				Vector3f X1 = X;
				Vector3f X0 = X;
				X1(a) = X(a) + pert;

				for (int b=0; b<3; b++) {
					Vector3f X11 = X1;
					Vector3f X10 = X1;
					Vector3f X01 = X0;
					Vector3f X00 = X0;

					X11(b) = X1(b) + pert;
					X01(b) = X0(b) + pert;

					Hxx(a,b) = (hamilton2(X11, U, lamd)
							- hamilton2(X10, U, lamd)
							- hamilton2(X01, U, lamd)
							+ hamilton2(X00, U, lamd)) / (pert2 * pert2);
				}
			}

			Matrix<double, 2, 2> Huu;
			for (int a=0; a<2; a++) {
				Vector2f U1 = U;
				Vector2f U0 = U;
				U1(a) = U(a) + pert;

				for (int b=0; b<2; b++) {
					Vector2f U11 = U1;
					Vector2f U10 = U1;
					Vector2f U01 = U0;
					Vector2f U00 = U0;

					U11(b) = U1(b) + pert;
					U01(b) = U0(b) + pert;

					Huu(a,b) = (hamilton2(X, U11, lamd)
							- hamilton2(X, U10, lamd)
							- hamilton2(X, U01, lamd)
							+ hamilton2(X, U00, lamd)) / (pert2 * pert2);
				}
			}

			Eigen::MatrixXd testMat(2, 2);
			testMat(0,0) = Huu(0,0);
			testMat(0,1) = Huu(0,1);
			testMat(1,0) = Huu(1,0);
			testMat(1,1) = Huu(1,1);

			// Compute the pseudo-inverse of the identity matrix
			Eigen::MatrixXd pseudoInv = pseudoInverse(testMat);

			Matrix<double, 2, 2> invHuu;
			Matrix<double, 3, 3> A;
			Matrix<double, 3, 3> B;
			Matrix<double, 3, 3> C;

			invHuu(0,0) = pseudoInv(0,0);
			invHuu(0,1) = pseudoInv(0,1);
			invHuu(1,0) = pseudoInv(1,0);
			invHuu(1,1) = pseudoInv(1,1);

			A = f_x - f_u * (invHuu * Hux);
			B = - f_u * (invHuu * f_u.transpose());
			C = Hxx - Hux.transpose() * (invHuu * Hux);

			// assign values to matP, matQ, matR, matS, matPbar, matQbar
			for (int k=0; k<M; k++) {

				double D_jk = b_D_mat[j][k];

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
						matPbar(a,3*k+a) = 2 * b_Qf(a,a);
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

		Vector3f temp_delX = desX - currX;
		Vector3d delX;
		delX(0) = static_cast<double> (temp_delX(0));
		delX(1) = static_cast<double> (temp_delX(1));
		delX(2) = static_cast<double> (temp_delX(2));

		Z0(0,0) = delX(0);
		Z0(1,0) = delX(1);
		Z0(2,0) = delX(2);

		Ze = - Inv_Ve * (V0 * Z0);

		Vector3d del_Lmbd;
		del_Lmbd(0) = Ze(3*M-3,0);
		del_Lmbd(1) = Ze(3*M-2,0);
		del_Lmbd(2) = Ze(3*M-1,0);

		// calculate gradient of f wrt u
		Matrix<double, 3, 2> f_u;
		for (int a=0; a<2; a++) {
			Vector2f desUper = desU;
			desUper(a) = desU(a) + pert;
			Vector3d dXper = dynamics2(desX, desUper);
			Vector3d dX = dynamics2(desX, desU);

			for (int i=0; i<3; i++) {
				f_u(i,a) = ( dXper(i) - dX(i) ) / pert2;
			}
		}

		// Calculation of Hessian of Hamilton
		Matrix<double, 2, 3> Hux;
		for (int a=0; a<2; a++) {
			Vector2f desUper = desU;
			desUper(a) = desU(a) + pert;

			for (int b=0; b<3; b++) {
				Vector3f desXper = desX;
				desXper(b) = desX(b) + pert;

				Hux(a,b) = ( hamilton2(desXper, desUper, des_lamd)
						- hamilton2(desX, desUper, des_lamd)
						- hamilton2(desXper, desU, des_lamd)
						+ hamilton2(desX, desU, des_lamd) ) / (pert2 * pert2);
			}
		}

		Matrix<double, 2, 2> Huu;
		for (int a=0; a<2; a++) {
			Vector2f desU1 = desU;
			Vector2f desU0 = desU;
			desU1(a) = desU(a) + pert;

			for (int b=0; b<2; b++) {
				Vector2f desU11 = desU1;
				Vector2f desU10 = desU1;
				Vector2f desU01 = desU0;
				Vector2f desU00 = desU0;

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

		Matrix<double, 2, 2> invHuu;
		invHuu(0,0) = invHuu_temp(0,0);
		invHuu(0,1) = invHuu_temp(0,1);
		invHuu(1,0) = invHuu_temp(1,0);
		invHuu(1,1) = invHuu_temp(1,1);

		Vector2d delU = - invHuu * ( Hux * delX + f_u.transpose() * del_Lmbd);

		Vector2f newU;
		newU(0) = desU(0) + static_cast<float>(delU(0));
		newU(1) = desU(1) + static_cast<float>(delU(1));

		T_sp = newU(0);
		Theta_sp = newU(1);

		// std::cout << time << " " << _local_pos.y << " " << desX(0) << " " << desX(1) << " " << desX(2) << " " << currX(0) << " " << currX(1) << " " << currX(2)
		//  << " " << Theta_sp << " " << pitch;

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
	std::cout << "Code is Running" << result[0] << " " << result[1] << std::endl;
	return result;
}
