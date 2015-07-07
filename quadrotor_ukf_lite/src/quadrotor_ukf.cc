#include "quadrotor_ukf.hh"

using namespace arma;

QuadrotorUKF::QuadrotorUKF()  
{
  // Init Dimensions
  stateCnt         = 14;
  procNoiseCnt     = 11;
  measNoiseSLAMCnt = 6; 
  measNoiseGPSCnt  = 9; 
  L = stateCnt + procNoiseCnt;
  // Init State
  xHist.clear();
  uHist.clear();
  xTimeHist.clear();
  Xa = zeros<mat>(stateCnt, 2*L+1);
  Va = zeros<mat>(procNoiseCnt, 2*L+1);
  P = zeros<mat>(stateCnt,stateCnt);
  P(0,0) = 0.5*0.5;
  P(1,1) = 0.5*0.5;
  P(2,2) = 0.1*0.1;
  P(3,3) = 0.1*0.1;
  P(4,4) = 0.1*0.1;
  P(5,5) = 0.1*0.1;
  P(6,6) = 10*PI/180*10*PI/180;
  P(7,7) = 10*PI/180*10*PI/180;
  P(8,8) = 10*PI/180*10*PI/180;
  P(9,9)   =  0.01*0.01;
  P(10,10) =  0.01*0.01;
  P(11,11) =  0.01*0.01;
  P(12,12) =  0.00001*0.00001;
  P(13,13) =  0.00001*0.00001;
  Rv = eye<mat>(procNoiseCnt,procNoiseCnt);
  // Init Sigma Points
  alpha = 0.1;
  beta  = 2;
  kappa = 0;
  GenerateWeights();
  // Other Inits
  g = 9.81;
  initMeasure = false;
  initGravity = false;
}

QuadrotorUKF::~QuadrotorUKF() { }
bool      QuadrotorUKF::isInitialized() { return (initMeasure && initGravity); }
colvec    QuadrotorUKF::GetState()           { return xHist.front();     }
ros::Time QuadrotorUKF::GetStateTime()       { return xTimeHist.front(); }
mat       QuadrotorUKF::GetStateCovariance() { return P;                 }
void      QuadrotorUKF::SetGravity(double _g) { g = _g; initGravity = true; }
void      QuadrotorUKF::SetImuCovariance(const mat& _Rv) { Rv = _Rv; }

void QuadrotorUKF::SetUKFParameters(double _alpha, double _beta, double _kappa)
{
  alpha = _alpha;
  beta  = _beta;
  kappa = _kappa;
  GenerateWeights();
}

void QuadrotorUKF::SetInitPose(colvec p, ros::Time time)
{
  colvec x = zeros<colvec>(stateCnt);
  x.rows(0,2)  = p.rows(0,2);
  x.rows(6,8)  = p.rows(3,5);
  xHist.push_front(x);
  uHist.push_front(zeros<colvec>(6));
  xTimeHist.push_front(time);
  initMeasure = true;
}

bool QuadrotorUKF::ProcessUpdate(colvec u, ros::Time time)
{
  if (!initMeasure || !initGravity)
    return false;
  // Just update state, defer covariance update
  double dt = (time-xTimeHist.front()).toSec();
  //xHist.front().print("xHist.front() = ");
  if(fabs(dt) > 1)
    dt = 0.01;
  //printf("dt ProcessModel = %lf\n", dt);
  colvec x = ProcessModel(xHist.front(), u, zeros<colvec>(procNoiseCnt), dt);
  //x.print("x = ProcessModel = ");
  xHist.push_front(x);
  uHist.push_front(u);
  xTimeHist.push_front(time);

  return true;
}

bool QuadrotorUKF::MeasurementUpdateSLAM(colvec z, mat RnSLAM, ros::Time time)
{
  // Init
  if (!initMeasure || !initGravity)
    return false;
  // A priori covariance
  //RnSLAM.print("RnSLAM = ");
  list<colvec>::iterator    kx;
  list<colvec>::iterator    ku; 
  list<ros::Time>::iterator kt;
  //printf("HERE1\n");fflush(NULL);
  PropagateAprioriCovariance(time, kx, ku, kt);
  colvec x = *kx;
  //printf("HERE2\n");fflush(NULL);
  // Get Measurement
  mat H = MeasurementModelSLAM();
  //printf("HERE3\n");
  colvec za = H * x;
  // Compute Kalman Gain
  mat S = H * P * trans(H) + RnSLAM;
  mat K = P * trans(H) * inv(S);
  //printf("HERE4\n");
  // Innovation
  colvec inno = z - za;
  //printf("HERE5\n");
  // Handle angle jumps
  inno(3) = asin(sin(inno(3)));
  //printf("HERE6\n");
  // Posteriori Mean
  x += K * inno;
  *kx = x;
  //printf("HERE7\n");
  //P.print("Before innovation P = ");
  // Posteriori Covariance
  P = P - K * H * P;
  //P.print("After innovation P = ");
  // Propagate Aposteriori State
  PropagateAposterioriState(kx, ku, kt);
  //printf("HERE8\n");
  return true;
}

bool QuadrotorUKF::MeasurementUpdateGPS(colvec z, mat RnGPS, ros::Time time)
{
  // Init
  if (!initMeasure || !initGravity)
    return false;
  // A priori covariance
  list<colvec>::iterator    kx;
  list<colvec>::iterator    ku; 
  list<ros::Time>::iterator kt;
  PropagateAprioriCovariance(time, kx, ku, kt);
  colvec x = *kx;
  // Get Measurement
  mat H = MeasurementModelGPS();
  colvec za = H * x;
  // Compute Kalman Gain
  mat S = H * P * trans(H) + RnGPS;
  mat K = P * trans(H) * inv(S);
  // Innovation
  colvec inno = z - za;
  // Handle angle jumps
  inno(6) = asin(sin(inno(6)));
  // Posteriori Mean
  x += K * inno;
  *kx = x;
  // Posteriori Covariance
  P = P - K * H * P;
  // Propagate Aposteriori State
  PropagateAposterioriState(kx, ku, kt);

  return true;
}

void QuadrotorUKF::GenerateWeights()
{
  // State + Process noise
  lambda = alpha*alpha*(L+kappa)-L;
  wm = zeros<rowvec>(2*L+1);
  wc = zeros<rowvec>(2*L+1);
  wm(0) = lambda / (L+lambda);
  wc(0) = lambda / (L+lambda) + (1-alpha*alpha+beta);
  for (int k = 1; k <= 2*L; k++)
  {
    wm(k) = 1 / (2 * (L+lambda));
    wc(k) = 1 / (2 * (L+lambda));
  }
  gamma = sqrt(L + lambda);
}

void QuadrotorUKF::GenerateSigmaPoints()
{
  // Expand state
  colvec x   = xHist.back();
  //x.print("x =");
  colvec xaa = zeros<colvec>(L);
  xaa.rows(0,stateCnt-1) = x;
  mat Xaa = zeros<mat>(L, 2*L+1);
  mat Paa = zeros<mat>(L,L);
  Paa.submat(0, 0, stateCnt-1, stateCnt-1) = P;
  Paa.submat(stateCnt, stateCnt, L-1, L-1) = Rv;


  //Rv.print("Rv = ");
  //P.print("P = ");
  //Paa.print("Paa = ");
  // Matrix square root
  mat sqrtPaa = trans(chol(Paa));
  // Mean
  Xaa.col(0) = xaa;
  mat xaaMat = repmat(xaa,1,L);
  //Xaa.print("Xaa 1=");
  Xaa.cols(  1,  L) = xaaMat + gamma * sqrtPaa;
  //Xaa.print("Xaa 2=");
  Xaa.cols(L+1,L+L) = xaaMat - gamma * sqrtPaa;
  // Push back to original state
  //Xaa.print("Xaa 3=");
  Xa = Xaa.rows(0, stateCnt-1);
  Va = Xaa.rows(stateCnt, L-1);
}

colvec QuadrotorUKF::ProcessModel(const colvec& x, const colvec& u, const colvec& v, double dt)
{
  //printf("ProcessModel Function\n");
  //x.print("x ProcessModel = ");
  //v.print("v ProcessModel = ");
  //u.print("u ProcessModel = ");
  //printf("dt ProcessModel = %lf\n", dt);
  mat R = ypr_to_R(x.rows(6,8));
  colvec ag(3);
  ag(0) = 0;
  ag(1) = 0;
  ag(2) = g;
  // Acceleration
  colvec a = u.rows(0,2) + v.rows(0,2);
  colvec ddx = R * (a - x.rows(9,11)) - ag;
  // Rotation
  colvec w = u.rows(3,5) + v.rows(3,5);
  mat dR = eye<mat>(3,3);
  dR(0,1) = -w(2) * dt;
  dR(0,2) =  w(1) * dt;
  dR(1,0) =  w(2) * dt;
  dR(1,2) = -w(0) * dt;
  dR(2,0) = -w(1) * dt;
  dR(2,1) =  w(0) * dt;
  mat Rt = R * dR;
  // State
  colvec xt = x;
  xt.rows(0,2)   = x.rows(0,2) + x.rows(3,5)*dt + ddx*dt*dt/2;
  xt.rows(3,5)   =               x.rows(3,5)    + ddx*dt     ;
  xt.rows(6,8)  = R_to_ypr(Rt);
  xt.rows(9,11)  = x.rows(9,11)  + v.rows(6,8) *dt;
  xt.rows(12,13) = x.rows(12,13) + v.rows(9,10)*dt;

  //xt.print("xt ProcessModel = ");

  return xt;
}

mat QuadrotorUKF::MeasurementModelSLAM()
{
  mat H = zeros<mat>(measNoiseSLAMCnt, stateCnt);
  H(0,0) = 1;
  H(1,1) = 1;
  H(2,2) = 1;
  H(3,6) = 1;
  H(4,7) = 1;
  H(5,8) = 1;
  H(4,12) = 1;
  H(5,13) = 1;
  return H;
}

mat QuadrotorUKF::MeasurementModelGPS()
{
  mat H = zeros<mat>(measNoiseGPSCnt, stateCnt);
  H(0,0) = 1;
  H(1,1) = 1;
  H(2,2) = 1;
  H(3,3) = 1;
  H(4,4) = 1;
  H(5,5) = 1;
  H(6,6) = 1;
  H(7,7) = 1;
  H(8,8) = 1;
  H(7,12) = 1;
  H(8,13) = 1;
  return H;
}

void QuadrotorUKF::PropagateAprioriCovariance(const ros::Time time, 
    list<colvec>::iterator& kx, list<colvec>::iterator& ku, list<ros::Time>::iterator& kt)
{
  // Find aligned state, Time
  double mdt = 1e300;
  //printf("Go 0.1\n");fflush(NULL);
  list<colvec>::iterator    k1 = xHist.begin();
  //printf("Go 0.2\n");fflush(NULL);
  list<colvec>::iterator    k2 = uHist.begin();
  //printf("Go 0.3\n");fflush(NULL);
  list<ros::Time>::iterator k3 = xTimeHist.begin();
  //printf("Go 0.4\n");fflush(NULL);
  int                       k4 = 0;
  //printf("Go 1\n");fflush(NULL);
  for (; k1 != xHist.end(); k1++, k2++, k3++, k4++)
  {
    //printf("Go 1.1\n"); fflush(NULL);
    double dt = fabs((*k3 - time).toSec());
    //printf("dt = %lf, mdt = %lf\n", dt, mdt);fflush(NULL);
    if (dt < mdt)
    {
      //printf("in the if clause 1\n"); fflush(NULL);
      mdt = dt; 
      kx  = k1;
      ku  = k2;
      kt  = k3;
    }
    else
    {
      //printf("in the if clause 2\n"); fflush(NULL);
      break;
    }
  }
  //printf("Go 2 - kx = %p, kp = %p \n", kx, kt);fflush(NULL);
  colvec    cx = *kx;
  ros::Time ct = *kt;
  //printf("Go 2.1\n");fflush(NULL);
  colvec    px = xHist.back();
  //printf("Go 2.2\n");fflush(NULL);
  ros::Time pt = xTimeHist.back();
  //printf("Go 2.3\n");fflush(NULL);
  double dt = (ct - pt).toSec();
  //printf("Go 3\n");fflush(NULL);
  if (fabs(dt) < 0.001)
  {
    kx = xHist.begin();
    ku = uHist.begin();
    kt = xTimeHist.begin();
    return;
  }
  //printf("Go 4\n");fflush(NULL);
  // Delete redundant states
  xHist.erase(k1, xHist.end());
  uHist.erase(k2, uHist.end());
  xTimeHist.erase(k3, xTimeHist.end());
  //printf("Go 5\n");fflush(NULL);
  // rot, gravity
  mat pR = ypr_to_R(px.rows(6,8));
  colvec ag(3);
  ag(0) = 0;
  ag(1) = 0;
  ag(2) = g;
  //printf("Go 6\n");fflush(NULL);
  // Linear Acceleration
  mat dv = cx.rows(3,5) - px.rows(3,5);
  colvec a = trans(pR) * (dv / dt + ag) + px.rows(9,11);
  //printf("Go 7\n");fflush(NULL);
  // Angular Velocity
  mat dR = trans(pR) * ypr_to_R(cx.rows(6,8));
  colvec w = zeros<colvec>(3);
  w(0) = dR(2,1) / dt;
  w(1) = dR(0,2) / dt;
  w(2) = dR(1,0) / dt;
  // Assemble state and control
  //printf("Go 8\n");fflush(NULL);
  colvec u = join_cols(a,w);
  // Generate sigma points
  GenerateSigmaPoints();
  //printf("Go 9\n");fflush(NULL);
  // Mean
  //Xa.print("Xa 1 = ");
  //u.print("u = ");
  //Va.print("Va = ");
  if(fabs(dt) > 1)
    dt = 0.01;
  for (int k = 0; k < 2*L+1; k++)
    Xa.col(k) = ProcessModel(Xa.col(k), u, Va.col(k), dt);
  //Xa.print("Xa 2 = ");
  //printf("Go 10\n");fflush(NULL);
  // Handle jump between +pi and -pi !
  double minYaw = as_scalar(min(Xa.row(6), 1));
  double maxYaw = as_scalar(max(Xa.row(6), 1));
  if (fabs(minYaw - maxYaw) > PI)
  {
    for (int k = 0; k < 2*L+1; k++)
      if (Xa(6,k) < 0)
        Xa(6,k) += 2*PI;
  }
  //printf("Go 11\n");fflush(NULL);
  // Now we can get the mean...
  //###Xa.print("Xa 3 = ");
  colvec xa = sum( repmat(wm,stateCnt,1) % Xa, 1 );
  // Covariance
  P.zeros();
  //P.print("P.zeros() = ");
  for (int k = 0; k < 2*L+1; k++)
  {
    colvec d = Xa.col(k) - xa;
    //xa.print("xa = ");
    //Xa.col(k).print("Xa.col(k) = ");
    //d.print("d = ");
    //printf("k = [%d]\n", k);
    P += as_scalar(wc(k)) * d * trans(d);
  }
  //P.print("P += ");
  //printf("Go 12\n");fflush(NULL);
  return;
}

void QuadrotorUKF::PropagateAposterioriState(list<colvec>::iterator kx, list<colvec>::iterator ku, list<ros::Time>::iterator kt)
{
  for (; kx != xHist.begin(); kx--, ku--, kt--)
  {
    list<colvec>::iterator _kx = kx;
    _kx--;
    list<colvec>::iterator _ku = ku;
    _ku--;
    list<ros::Time>::iterator _kt = kt;
    _kt--;
    //(*kx).print("*kx = ");
    //cout << "Seconds = " << (*_kt - *kt).toSec() << endl;
    *_kx = ProcessModel(*kx, *_ku, zeros<colvec>(procNoiseCnt), (*_kt - *kt).toSec());
    //(*_kx).print("*_kx = ");
  }

}

