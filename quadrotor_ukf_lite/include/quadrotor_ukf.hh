#ifndef __QUADROTOR_UKF_HH__
#define __QUADROTOR_UKF_HH__

#include <list>
#include <cmath>
#include <vector>
#include <string>
#include <iostream>
#include <algorithm>
#include <armadillo>

#include <ros/ros.h>

#include <pose_utils.h>

using namespace arma;

class QuadrotorUKF
{
	private:

		// State History and Covariance
		list<colvec>    xHist;
		list<colvec>    uHist;
		list<ros::Time> xTimeHist;
		mat P;

		// Process Covariance Matrix
		mat Rv;

		// Instance sigma points
		mat Xa;
		mat Va;

		// Initial process update indicator
		bool initMeasure;
		bool initGravity;

		// Dimemsions
		int stateCnt;
		int procNoiseCnt;
		int measNoiseSLAMCnt;
		int measNoiseGPSCnt;
		int L;

		// Gravity
		double g;

		// UKF Parameters
		double alpha;
		double beta;
		double kappa;
		double lambda;
		double gamma;
		// UKF Weights
		rowvec wm;
		rowvec wc;

		// Private functions
		void GenerateWeights();
		void GenerateSigmaPoints();
		colvec ProcessModel(const colvec& x, const colvec& u, const colvec& v, double dt);
		mat MeasurementModelSLAM();
		mat MeasurementModelGPS();
		void PropagateAprioriCovariance(const ros::Time time, list<colvec>::iterator& kx, list<colvec>::iterator& ku, list<ros::Time>::iterator& kt);
		void PropagateAposterioriState(list<colvec>::iterator kx, list<colvec>::iterator ku, list<ros::Time>::iterator kt);

	public:

		QuadrotorUKF();
		~QuadrotorUKF();

		bool      isInitialized();
		colvec    GetState();
		ros::Time GetStateTime();
		mat       GetStateCovariance();

		void SetGravity(double _g);
		void SetImuCovariance(const mat& _Rv);
		void SetUKFParameters(double _alpha, double _beta, double _kappa);
		void SetInitPose(colvec p, ros::Time time);

		bool ProcessUpdate(colvec u, ros::Time time);
		bool MeasurementUpdateSLAM(colvec z, mat RnSLAM, ros::Time time);
		bool MeasurementUpdateGPS(colvec z, mat RnGPS, ros::Time time);
};

#endif
