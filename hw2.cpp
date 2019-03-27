#include <iostream>
#include <string>
#include <fstream>
#include <cstdlib>
#include <cmath>
#include <ilcplex\ilocplex.h>

using namespace std;

void solve11_9b(){
	IloEnv env;
	IloArray<IloBoolVar> x(env,6);

	for(int i=0;i<6;i++){
		ostringstream stm;
		stm<<"x"<<"["<<i+1<<"]";
		x[i]=IloBoolVar(env,stm.str().c_str());
	}

	IloRangeArray constraint(env);
	
	constraint.add(x[0]+x[1]>=1);
	constraint.add(x[1]+x[2]>=1);
	constraint.add(x[0]+x[3]>=1);
	constraint.add(x[1]+x[4]>=1);
	constraint.add(x[2]+x[4]>=1);
	constraint.add(x[2]+x[5]>=1);
	constraint.add(x[3]+x[4]>=1);
	constraint.add(x[4]+x[5]>=1);
	
	IloExpr objective(env);
	int c[6]={40,65,43,48,72,36};
	for(int i=0;i<6;i++)
		objective+=c[i]*x[i];

	IloModel model(env);
	model.add(constraint);
	model.add(IloMinimize(env,objective));
	IloCplex cplex(model);
	cplex.exportModel("problem11-9b.lp");
	cplex.solve();

	cout<<"Solution status = " << cplex.getStatus()<<endl;
	cout<<"Objective function value = "<<cplex.getObjValue()<<endl;
	cout<<"Optimal solutions = "<<endl;
	for(int i=0;i<6;i++)
		cout<<"x["<<i+1<<"] = "<< cplex.getValue(x[i])<<" ";
	cout<<endl;
	env.end();
}
void solve11_9d(){
	IloEnv env;
	IloArray<IloBoolVar> x(env,6);
	IloArray<IloBoolVar> y(env,8);
	for(int i=0;i<6;i++){
		ostringstream stm;
		stm<<"x"<<"["<<i+1<<"]";
		x[i]=IloBoolVar(env,stm.str().c_str());
	}
	for(int i=0;i<8;i++){
		ostringstream stm;
		stm<<"y"<<"["<<i+1<<"]";
		y[i]=IloBoolVar(env,stm.str().c_str());
	}
	IloRangeArray constraint(env);
	IloExpr con1(env);
	for(int i=0;i<6;i++)
		con1+=x[i];
	constraint.add(con1==2);
	int arr[][2]={{0,1},{1,2},{0,3},{1,4},{2,4},{2,5},{3,4},{4,5}};
	for(int i=0;i<8;i++){
		//constraint.add(x[arr[i][0]]-y[i]<=0);
		//constraint.add(x[arr[i][1]]-y[i]<=0);
		constraint.add(x[arr[i][0]]+x[arr[i][1]]-y[i]>=0);
	}
	IloExpr objective(env);
	for(int i=0;i<8;i++)
		objective+=y[i];

	IloModel model(env);
	model.add(constraint);
	model.add(IloMaximize(env,objective));
	IloCplex cplex(model);
	cplex.exportModel("problem11-9d.lp");
	cplex.solve();

	cout<<"Solution status = " << cplex.getStatus()<<endl;
	cout<<"Objective function value = "<<cplex.getObjValue()<<endl;
	cout<<"Optimal solutions = "<<endl;
	for(int i=0;i<6;i++)
		cout<<"x["<<i+1<<"] = "<< cplex.getValue(x[i])<<" ";
	cout<<endl;
	for(int i=0;i<8;i++)
		cout<<"y["<<i+1<<"] = "<< cplex.getValue(y[i])<<" ";
	cout<<endl;
				
	env.end();
}
void solve11_14(){
	IloEnv env;
	IloArray<IloArray<IloBoolVar>> x(env,4);

	for(int i=0;i<4;i++){
		x[i]=IloArray<IloBoolVar>(env,4);
		for(int j=0;j<4;j++){
			ostringstream stm;
			stm<<"x"<<"["<<i+1<<"]["<<j+1<<"]";
			x[i][j]=IloBoolVar(env,stm.str().c_str());
		}
	}
	
	IloRangeArray constraint(env);
	for(int i=0;i<4;i++){
		IloExpr con1(env);
		IloExpr con2(env);
		for(int j=0;j<4;j++){
			con1+=x[i][j];
			con2+=x[j][i];
		}
		constraint.add(con1==1);
		constraint.add(con2==1);
	}	
	int arr[][4]={{80,65,83,77},{54,87,61,66},{92,45,53,59},{70,61,81,76}};
	IloExpr objective(env);
	for(int i=0;i<4;i++){
		for(int j=0;j<4;j++){
			objective+=arr[i][j]*x[i][j];
		}
	}
	IloModel model(env);
	model.add(constraint);
	model.add(IloMaximize(env,objective));
	IloCplex cplex(model);
	cplex.exportModel("problem11-14.lp");
	cplex.solve();

	cout<<"Solution status = " << cplex.getStatus()<<endl;
	cout<<"Objective function value = "<<cplex.getObjValue()<<endl;
	cout<<"Optimal solutions = "<<endl;
	for(int i=0;i<4;i++){
		for(int j=0;j<4;j++)
			cout<<"x["<<i+1<<"]["<<j+1<<"] = "<< cplex.getValue(x[i][j])<<" ";
		cout<<endl;
	}			
	env.end();
}
int main(){
	solve11_9b();
	solve11_9d();
	solve11_14();
	return 0;
}