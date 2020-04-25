#ifndef DWA_H
#define DWA_H

#include <iostream>
#include <vector>
#include <math.h>

namespace DWA{

	struct Rect
	{
		float xmin, xmax;
		float ymin, ymax;
	};

	struct Config
	{
		float max_v, min_v;
		float max_w, min_w;
		float max_acc, max_dw;
		float v_reso, w_reso;
		float dt, T;

		float heading;
		float clearance;
		float velocity;
		Rect base;	//local map range (rectangle)
	};

	struct Velo
	{
		float liner;
		float angular;
	};

	struct Point
	{
		float x, y;
	};

	struct PointSet
	{
		std::vector<Point> points;
	};

	struct Pose
	{
		Point point;
		float theta;
	};


	struct DynamicWindow
	{
		int nPossibleV;
		std::vector<float> possibleV;
		int nPossibleW;
		std::vector<float> possibleW;
	};

	static void createDynamicWindow(Velo velo, Config cfg, DynamicWindow *dw)
	{
		float minV = fmaxf(cfg.min_v, velo.liner - cfg.max_acc*cfg.dt);
		float maxV = fminf(cfg.max_v, velo.liner + cfg.max_acc*cfg.dt);
		float minW = fmaxf(cfg.min_w, velo.angular - cfg.max_dw*cfg.dt);
		float maxW = fminf(cfg.max_w, velo.angular + cfg.max_dw*cfg.dt);

		int nPossibleV = (int)((maxV - minV)/cfg.v_reso);
		int nPossibleW = (int)((maxW - minW)/cfg.w_reso);

		dw = new DynamicWindow;	//create memory space
		dw->nPossibleV = nPossibleV;
		dw->nPossibleW = nPossibleW;

		//create control input space (discritized)
		for(int i = 0; i < dw->nPossibleV; i++){
			float _v = minV + (float)i*cfg.v_reso;
			dw->possibleV.push_back(_v);
		}

		for(int i = 0; i < dw->nPossibleW; i++){
			float _w = minW + (float)i*cfg.w_reso;
			dw->possibleW.push_back(_w);
		}
	}

	//Motion model Xt+1=f(Xt, Ut)
	static Pose motion(Pose X, Velo U, float T)
	{
		Pose _X;
		_X.theta = X.theta + U.angular*T;
		_X.point.x = X.point.x + U.liner*cosf(_X.theta)*T;
		_X.point.y = X.point.y + U.liner*sinf(_X.theta)*T;
		return _X;
	}

	static float calcVelocityCost(Velo U, Config cfg)
	{
		return cfg.max_v - U.liner;
	}

	static float calcHeadingCost(Pose X, Point goal)
	{
		float dx = goal.x - X.point.x;
		float dy = goal.y - X.point.y;
		float dyaw = atan2(dy, dx);
		float c = dyaw - X.theta;
		return fabs(atan2(sin(c), cos(c)));
	}

	static float calcCrearnaceCost(Pose X, Velo U, PointSet set_obs, Config cfg)
	{
		float t = 0.0;
		Pose _X = X;
		float minr = INFINITY;

		while(t < cfg.T){
			_X = motion(_X, U, cfg.dt);

			for(int i = 0; i < set_obs.points.size(); i++){
				float dx = _X.point.x - set_obs.points[i].x;
				float dy = _X.point.y - set_obs.points[i].y;

				float x = -dx*cos(_X.theta) - dy*sin(_X.theta);
				float y = -dx*(-sin(_X.theta)) - dy*cos(_X.theta);

				if (x <= cfg.base.xmax &&
						x >= cfg.base.xmin &&
						y <= cfg.base.ymax &&
						y >= cfg.base.ymin){
					return INFINITY;
				}

				float r = sqrtf(dx*dx + dy*dy);
				if(r < minr){
					minr = r;
				}
			}
			t += cfg.dt;
		}
		return 1.0/minr;
	}

	static Velo planning(Pose X, Velo U, Point goal, PointSet obs, Config cfg)
	{
		//Create Dynamic Window
		DynamicWindow *dw;
		createDynamicWindow(U, cfg, dw);

		//Evaluate
		float total_cost = INFINITY;
		float cost;
		Velo bV;

		for(int v = 0; v < dw->nPossibleV; v++){
			for(int w = 0; w < dw->nPossibleW; w++){

				Velo tmp_velo;
				tmp_velo.liner = dw->possibleV[v];
				tmp_velo.angular = dw->possibleW[w];

				//calculate next pose by motion model
				Pose _X = motion(X, U, cfg.T);

				//calculate cost function
				cost =
						cfg.velocity*calcVelocityCost(tmp_velo, cfg)
						+ cfg.heading*calcHeadingCost(_X, goal)
						* cfg.clearance*calcCrearnaceCost(X, tmp_velo, obs, cfg);

				if(cost < total_cost){
					total_cost = cost;
					bV = tmp_velo;
				}
			}
		}

		delete dw;
		return bV;

	}
}

#endif
