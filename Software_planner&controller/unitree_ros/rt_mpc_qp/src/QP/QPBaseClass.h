/**
QPBaseClass.h

Description:	Header file of QPBaseClass

@Version:	1.0
@Author:	Chengxu Zhou (zhouchengxu@gmail.com)
@Release:	Thu 07 Apr 2016 07:08:57 PM CEST
@Update:	Mon 11 Apr 2016 02:48:28 PM CEST
*/
#pragma once

#include <list>
#include <map>
#include <Eigen/Core>
#include <boost/shared_ptr.hpp>
// #include "utils/utils.h"

class QPsolver;

class QPBaseClass
{
public:
	QPBaseClass(const std::string& qpSolverName = "EiQuadProg");
	virtual ~QPBaseClass() {};

	// call this first before anything else
	void initQP();
	void resizeQP(int nVars, int nEqCon, int nIneqCon);

	bool solveQP();

	int finalizeCostRows(const std::string &name, int r);
	int finalizeIneqRows(const std::string &name, int r);

	int addCostRows(const int& r);
	int addIneqRows(const int& r);

	inline const Eigen::VectorXd& getVars(){return _X;};

	inline int getNVars() const { return _nVars; }
	inline int getCostRowIdx() const { return _costRowIdx; }
	inline int getIneqRowIdx() const { return _ineqRowIdx; }

	std::string _solver_name;
	bool debug;

protected:
	static const int MAX_ROWS = 300;
	static const int MAX_VARS = 60;
	static const int MAX_N_INEQ = 300;

	int _nVars;
	int _nEqCon;
	int _nIneqCon;

	int _costRowIdx;
	int _ineqRowIdx;

	Eigen::MatrixXd _G;
	Eigen::VectorXd _g0;
	Eigen::VectorXd _X;
	Eigen::MatrixXd _A;
	Eigen::VectorXd _b;
	Eigen::MatrixXd _CE;
	Eigen::VectorXd _ce0;
	Eigen::MatrixXd _CI;
	Eigen::VectorXd _ci0;

private:
	boost::shared_ptr<QPsolver> _qpsolver;


};
