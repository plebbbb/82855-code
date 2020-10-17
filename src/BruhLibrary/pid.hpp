#include "main.h"
#include "utilityfunctions.hpp"
#include "globalvar.hpp"
using namespace pros;
#pragma once

  //curveS: a single S curve
  //Constraints(suggested): range: 0 to 50, upwards.  max height 127;
  //TBD, maybe actually calculate proper motion curves later and emulate them with curveS
class curveS{
  double* vars;
public:
  curveS(double arr[]){vars = arr;}
  double getval(double pos){
    return (vars[0]-vars[3])/(1+pow(M_E,-vars[1]*(pos-vars[2]))) + vars[3];
  }
};

  //dualScurve: a set of 2 S curves made to aproximate what motion profiling might look like
  /*dualScurve should extend curveS, also should be an interface ngl so we can use both one or two
  the current pointer access method is ok for these b/c they dont store anything motor-specific,
  its literally just a formula*/
struct dualScurve{
  curveS* a = NULL;
  curveS* b = NULL;
  dualScurve(curveS c, curveS d){a=&c; b=&d;}
  dualScurve(curveS c){a=&c;}
  double getval(double pos){
    if (pos < 50) return a->getval(pos);
    else if (b == NULL) return a->getval(50-pos);
    return b->getval(50-pos);
    //for the part past 50%, we flip the curveS, so that we only need one curve from 0-50 for each different profile
  };
};

  //beziernp: a candidate approach for our path finding solution
  /*This is a full on proper n-point bezier curve, where we can add
  as many transformations as we want but only 2 garanteed target locations
  */
/*class beziernp{
private:
  double** coords;
  double* binomialfactors; //double may be too small ngl
  int size; //coordinate size
public:
  //sinfo and einfo format: x, y, angle(rads), einfo 4th param is target end heading
  beziernp(double sinfo[3], double einfo[3], double offsetcoords[][2]){
    size = sizeof(offsetcoords)/sizeof(double[2])+4;
    coords = (double**)new double[size][2]; //how does casting to double** work here, i have no clue
    coords[0][0] = sinfo[0];
    coords[0][1] = sinfo[1];
    /*below: these values are here to prevent harsh turns due to existing momentum
    by applying a transformation in the current direction of movement*//*
    coords[1][0] = sinfo[0]+cos(sinfo[2])*estspd*vscalefac;
    coords[1][1] = sinfo[1]+sin(sinfo[2])*estspd*vscalefac;
    coords[size-1][0] = sinfo[0]+cos(sinfo[2])*127*vscalefac; //127 is a placeholder
    coords[size-1][1] = sinfo[1]+sin(sinfo[2])*127*vscalefac;
    coords[size][0] = einfo[0];
    coords[size][1] = einfo[1];
    std::copy(offsetcoords[0],offsetcoords[size-3],coords[1]);//I'm honestly not too sure this works, pls test in eclipse or something
    //above: manual array size calc here cuz idk how to cast this one for .size()
  }
  beziernp(double sinfo[3], double einfo[3]){
    coords = (double**)new double[4][2];
    size = 4;
    coords[0][0] = sinfo[0];
    coords[0][1] = sinfo[1];
    /*below: these values are here to prevent harsh turns due to existing momentum
    by applying a transformation in the current direction of movement*//*
    coords[1][0] = sinfo[0]+cos(sinfo[2])*estspd*vscalefac;
    coords[1][1] = sinfo[1]+sin(sinfo[2])*estspd*vscalefac;
    coords[size-1][0] = sinfo[0]+cos(sinfo[2])*127*vscalefac; //127 is a placeholder
    coords[size-1][1] = sinfo[1]+sin(sinfo[2])*127*vscalefac;
    coords[size][0] = einfo[0];
    coords[size][1] = einfo[1];
  }
/*  //this getvals assumes we maintain the global pointer target coordinate approach
  //www.desmos.com/calculator/cahqdxeshd
  void getval(double it){
    double xtot = 0;
    double ytot = 0;
    //iteration loop for n-amount of offset points, pls confirm if working
    //double it ranges from 0 to 1
    for (int i = 0; i < size-2; i++){
      xtot+=(1-it)*((1-it)*coords[i][0]+it*coords[i+1][0])+it*((1-it)*coords[i+1][0]+it*coords[i+2][0]);
      ytot+=(1-it)*((1-it)*coords[i][1]+it*coords[i+1][1])+it*((1-it)*coords[i+1][1]+it*coords[i+2][1]);
    }
    xtot*=(1-it);
    ytot*=(1-it);
    xyaT[0] = xtot;
    xyaT[1] = ytot;
  }
  */
  //variation two, probably higher chance of working than the other option, but probably
  //a lot more taxing, we may actually hit performance issues from this once
  //https://www.desmos.com/calculator/xlpbe9bgll
  /*
  void getvalF(double t){
    double x = 0; double y = 0;
    for (int i = 0; i < size; i++){
      double ccfactor = getCCF(t,i);
      x+=ccfactor*coords[i][0];
      y+=ccfactor*coords[i][1];
    }
    xyaT[0] = x;
    xyaT[1] = y;
  }
  //formatting: t is iteration position, k is index, v is coordinate
  double getCCF(double t, double k){
    return binomialfactors[(int)k]*pow((1-t),size-1-k)*pow(t,k); //to save performance we calc this once per coord
  }
  //in order to minimize computations, we only calculate binomial factors once upon creation of instances
  //even better would be to precompute all of them globally but I need something localized for testing rn
  void computebinomialfactor(){
    binomialfactors = new double[size];
    for (int i = 0; i < size; i++){
      binomialfactors[i] = getbifac(size, i);
    }
  }
  //this part is expensive, if it's too much we might try precalculating it somehow,
  //also unsigned longs for safety's sake, probably big enough unless we dump some stupid amount of points
  double getbifac(double n, double k){
    //return factorial(n)/(factorial(k)*factorial(n-k));
    //above is the standard approach, below is optimized approach
    //https://cp-algorithms.com/combinatorics/binomial-coefficients.html#toc-tgt-0
    double ft = 1;
    for (int i = 1; i < k; i++){
      ft = ft*(n-k+i)/i; //this approach does it directly on the fraction, minimizing issues of numbers being too big
    }
    return ft;
  }
};

//compositebezier: another candidate approach for path finding
/*This is a piecewise bezier curve approach using beziernp instances, of which
will have 4 default offset points each, two for end points, two for heading targets*//*
struct compositebezier{
  std::vector<beziernp> beziers; //ngl shoulda started running vectors a lot sooner
  int size;
  //arr config: include the bezier from your current positon too
  compositebezier(beziernp arr[]){
    size = sizeof(arr)/sizeof(beziernp);
    for (int i = 0; i < size; i++) beziers.push_back(arr[i]);
  }
  //mvcoords config - do not include the current position, only future positions
  //new bezier curve is generated using current point
  compositebezier(double mvcoords[][3]){
    size = sizeof(mvcoords)/sizeof(double[3])+1;
    beziers.push_back(beziernp(mvcoords[0],mvcoords[0]));
    for (int i = 1; i < size; i++){
      beziers.push_back(beziernp(mvcoords[0],mvcoords[1]));
    }
  }
};
*/

/*PID: generic PID system*/
//NOTE: HAS NOT BEEN TESTED PLS TEST
//ANOTHER NOTE: DEFAULT TGT = 0
class PID{
private:
  /*Integral mode configurations:
  false: Direct I scaling - 100% of I is added each cycle
  true: Asymptope I - I approaches the max*/
  bool Imode = 0;

  /*Proportional mode configurations
  false: Raw input
  true: External S curve, percent to target based
  */
  bool Pmode = 0;

  //Izerocutoff: turns I to 0 when target reached
  bool Izerocutoff = true;

  //Pk, Ik, Dk, and I fraction array
  double* ratios;

  //selected dualScurve for Pmode = 1
  dualScurve* Scurve;

  //the max limits for the loop
  double maxIlimit;
  double maxlimit;

  //P, I, D, position, and target array
  double PIDa[4] = {0};

  double lasterror = 0;
public:
  PID(double scalers[], bool ms[], double limits[]){
    ratios = scalers; Pmode = ms[0]; Imode = ms[1]; Izerocutoff = ms[2]; maxlimit = limits[0]; maxIlimit = limits[1];
  }
  //note: if dualScurve is to be used, input percentage to target values
  PID(double scalers[], bool ms[], double limits[], dualScurve curve){
    ratios = scalers; Scurve = &curve; Pmode = ms[0]; Imode = ms[1]; Izerocutoff = ms[2]; maxlimit = limits[0]; maxIlimit = limits[1];
  }
  //sets a new target for the loop w/o resetting PIDa
  void set_tgt_soft(double tgt){
    PIDa[3] = tgt;
  }
  //sets a new target for the loop and resets PIDa
  void set_tgt_clean(double tgt){
    PIDa[0]=0;PIDa[1]=0;PIDa[2]=0; //this is stupid
    PIDa[3] = tgt;
  }
  double update(double in){
    double err = (PIDa[3]-in);
    if (Pmode) PIDa[0] = isposorneg(err)*Scurve->getval(fabs(err));
    else PIDa[0] = err;
    if (Imode) PIDa[1] += (err-PIDa[1])/ratios[3];
    else PIDa[1] += err;
    if (Izerocutoff == true && round(in*10) == 0) PIDa[1] = 0;
    if (fabs(PIDa[1]) > maxIlimit) PIDa[1] = isposorneg(PIDa[1])*maxIlimit;
    PIDa[2] = err-lasterror;
    lasterror = err;
    double final = PIDa[0]*ratios[0] + PIDa[1]*ratios[1] + PIDa[2]*ratios[2];
    return isposorneg(final)*determinesmallest(fabs(final),maxlimit);
  };
};
