/*****************************************************************************
QPBaseClass.cpp

Description:    source file of QPBaseClass

@Version:   1.0
@Author:    Chengxu Zhou (zhouchengxu@gmail.com)
@Release:   Thu 07 Apr 2016 07:09:04 PM CEST
@Update:    Mon 13 Jun 2016 12:11:31 PM CEST
*****************************************************************************/
#include "QP/QPBaseClass.h"
#include "utils/EiQuadProg/EiQuadProg.hpp"
#include <iostream>

inline bool is_bad_num(const double& num)
{
	return std::isinf(num) || std::isnan(num);
}

class QPsolver
{
public:
	QPsolver() {};
	virtual ~QPsolver() {};
	virtual void resize(const int& nVar, const int& nEq, const int& nIneq) = 0;
	virtual void solve(Eigen::MatrixXd & G, const Eigen::VectorXd & g0, const Eigen::MatrixXd & CE, const Eigen::VectorXd & ce0, const Eigen::MatrixXd & CI, const Eigen::VectorXd & ci0, Eigen::VectorXd& X) = 0;

protected:
	int _nVar;
	int _nEq;
	int _nIneq;

};

//-------------------------------------------------------------------
class QPsolver_EiQuadProg : public QPsolver
{
public:
	QPsolver_EiQuadProg()
	{
		//std::cout << "Using EiQuadProg as QP solver...\n";
	};

	void resize(const int& nVar, const int& nEq, const int& nIneq)
	{
		_qpsolver.resize(nVar, nEq, nIneq);
	};

	void solve(Eigen::MatrixXd & G,
	           const Eigen::VectorXd & g0,
	           const Eigen::MatrixXd & CE,
	           const Eigen::VectorXd & ce0,
	           const Eigen::MatrixXd & CI,
	           const Eigen::VectorXd & ci0,
	           Eigen::VectorXd& X)
	{
		_qpsolver.solve_quadprog(G, g0, CE, ce0, CI, ci0, X);
	};

private:
	Eigen::QP _qpsolver;
};

//-------------------------------------------------------------------
QPBaseClass::QPBaseClass(const std::string& qpSolverName)
	: _solver_name(qpSolverName)
{
	if (_solver_name == "EiQuadProg") {
		_qpsolver.reset(new QPsolver_EiQuadProg());
	}
	else {
		//COUT("please define qpsolver: qpOASES or EiQuadProg");
		assert(!"please define qpsolver: qpOASES or EiQuadProg");
		exit(0);
	}

	_nVars = 0;
	_nEqCon = 0;
	_nIneqCon = 0;

	_costRowIdx = 0;
	_ineqRowIdx = 0;
	_A.resize(MAX_ROWS, MAX_VARS);
	_b.resize(MAX_ROWS);

}

void QPBaseClass::initQP()
{
	_costRowIdx = 0;
	_ineqRowIdx = 0;

	_A.setZero();
	_b.setZero();
	_X.setZero();
	_CE.setZero();
	_ce0.setZero();
	_CI.setZero();
	_ci0.setZero();
}

void QPBaseClass::resizeQP(int nVars, int nEqCon, int nIneqCon)
{
	if (_nVars == nVars && _nEqCon == nEqCon && _nIneqCon == nIneqCon)
		return;

	_nVars = nVars;
	_nEqCon = nEqCon;
	_nIneqCon = nIneqCon;

	assert(_nVars <= MAX_VARS);
	assert(_nIneqCon <= MAX_N_INEQ);

	_qpsolver->resize(_nVars, nEqCon, nIneqCon);
	_G.resize(_nVars, _nVars);
	_g0.resize(_nVars);
	_CE.resize(_nVars, nEqCon);
	_ce0.resize(nEqCon);
	_CI.resize(_nVars, nIneqCon);
	_ci0.resize(nIneqCon);
	_X.resize(_nVars);

	debug = false;
}

bool QPBaseClass::solveQP()
{
    bool solveQP_successx;
    
	assert(_nIneqCon >= _ineqRowIdx);

	// _G = _A.block(0, 0, _costRowIdx, _nVars).transpose() * _A.block(0, 0, _costRowIdx, _nVars);
	// _g0 = -_A.block(0, 0, _costRowIdx, _nVars).transpose() * _b.segment(0, _costRowIdx);

	_qpsolver->solve(_G, _g0, _CE, _ce0, _CI, _ci0, _X);

	for (int i = 0; i < _nVars; i++)
    {
        if (!std::isnan(_X[i]))
        {
            solveQP_successx = true;
        }
        else
        {
            solveQP_successx = false;
            break;
        }
    }
		//assert(!std::isnan(_X[i]));
    
    return solveQP_successx;

}

int QPBaseClass::finalizeCostRows(const std::string &name, int r)
{
	if (r <= 0)
		return _costRowIdx;

	for (int i = _costRowIdx; i < _costRowIdx + r; i++) {
		if (is_bad_num(_b[i])) {
			std::cout << "bad cost term: " << name << ", " << i - _costRowIdx << "th member: " << _b[i] << std::endl;
			assert(false);
			return -1;
		}
	}
	_costRowIdx += r;
	assert(_costRowIdx <= _A.rows());
	return _costRowIdx;
}

int QPBaseClass::finalizeIneqRows(const std::string &name, int r)
{
	if (r <= 0)
		return _ineqRowIdx;

	for (int i = _ineqRowIdx; i < _ineqRowIdx + r; i++) {
		if (is_bad_num(_ci0[i])) {
			std::cout << "bad ineq term: " << name << ", " << i - _ineqRowIdx << "th member: " << _ci0[i] << std::endl;
			assert(false);
			return -1;
		}
	}
	_ineqRowIdx += r;
	assert(_ineqRowIdx <= _nIneqCon);
	return _ineqRowIdx;
}

int QPBaseClass::addCostRows(const int& r)
{
	if (r <= 0)
		return _costRowIdx;

	_costRowIdx += r;
	assert(_costRowIdx <= _A.rows());
	return _costRowIdx;
}

int QPBaseClass::addIneqRows(const int& r)
{
	if (r <= 0)
		return _ineqRowIdx;

	_ineqRowIdx += r;
	assert(_ineqRowIdx <= _nIneqCon);
	return _ineqRowIdx;
}
