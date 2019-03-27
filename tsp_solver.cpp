#define TOFLT(x) (static_cast<float>(x))
#define TOINT(x) (static_cast<int>(x))
#define MAXCOUNT 1048576
#include <iostream>
#include <string>
#include <fstream>
#include <cstdlib>
#include <cmath>
#include <ilcplex\ilocplex.h>

using namespace std;

int **points;
bool **mat;
int n;
ofstream of;
int dist(int *x,int *y){
	float d = sqrt(pow(TOFLT(x[1]-y[1]),2)+pow(TOFLT(x[2]-y[2]),2));
	return TOINT(d);
}
void ReadData();
void DeleteData();
void solveLP(IloCplex &cplex,int type);
void construct_model1();
void construct_model2();
void construct_model3_1();
void construct_model3_2();
bool check_conn(IloEnv &env,IloCplex &cplex,IloArray<IloArray<IloBoolVar>> &var_x,IloRangeArray &constraint);
int next(int cur);
int findf(bool *visit);
int sum(int *root);

int main(){
	ReadData();
	construct_model1();
	construct_model2();
	construct_model3_2();
	construct_model3_1();
	DeleteData();
	return 0;
}

void ReadData(){
	cin>>n;
	points = new int*[n];
	mat=new bool*[n];
	for(int i=0;i<n;i++){
		points[i]=new int[3];
		mat[i]=new bool[n];
		cin>>points[i][0]>>points[i][1]>>points[i][2];
	}
	of.open("out.txt");
}
void DeleteData(){
	for(int i=0;i<n;i++){
		delete points[i];
		delete mat[i];
	}
	delete points;
	delete mat;
	of.close();
}
void solveLP(IloCplex &cplex,int type){
	static int ver=1;
	static int prev=1; 
	char tmp[10];
	string s("TSP_sol_ver");
	s.append(_itoa(type,tmp,10));
	if(1==type){
		s.append(".");
		s.append(_itoa(ver,tmp,10));
	}
	s.append(".lp");
	cplex.exportModel(s.c_str());
	cplex.solve();
	ver++;
}
void construct_model1(){
	IloEnv env;
	IloArray<IloArray<IloBoolVar>> var_x(env,n);

	for(int i=0;i<n;i++){
		var_x[i]=IloArray<IloBoolVar>(env,n);
		for(int j=0;j<n;j++){
			ostringstream stm;
			stm<<"x"<<"["<<i<<"]["<<j<<"]";
			var_x[i][j]=IloBoolVar(env,stm.str().c_str());
		}
	}
	IloRangeArray constraint(env);
	for(int i=0;i<n;i++){
		IloExpr con1(env);
		for(int j=0;j<n;j++){
			if(j==i) continue;
			con1+=var_x[i][j];
		}
		constraint.add(con1==1);
	}
	for(int j=0;j<n;j++){
		IloExpr con2(env);
		for(int i=0;i<n;i++){
			if(j==i) continue;
			con2+=var_x[i][j];
		}
		constraint.add(con2==1);
	}

	IloExpr objective(env);
	for(int i=0;i<n;i++)
		for(int j=0;j<n;j++){
			if(i==j) continue;
			objective+=dist(points[i],points[j])*var_x[i][j];
		}


	int i;
	for(i=0;i<MAXCOUNT;i++){
		IloModel model(env);
		model.add(constraint);
		model.add(IloMinimize(env,objective));
		IloCplex cplex(model);
		solveLP(cplex,1);

		if(check_conn(env,cplex,var_x,constraint)){
			of<<"Solution status = " << cplex.getStatus()<<endl;
			of<<"Objective function value = "<<cplex.getObjValue()<<endl;
			of<<"Optimal solutions = "<<endl;
			for(int k=0;k<n;k++){
				for(int j=0;j<n;j++){
					if(mat[k][j]==1) of<<"x["<<k<<"]["<<j<<"] = "<<mat[k][j]<<" ";
				}
				of<<endl;
			}
			break;
		}
	}
	if(i==MAXCOUNT) of<<"Solution status = NO possible solution"<<endl;
	env.end();
}
void construct_model2(){
	IloEnv env;
	IloArray<IloArray<IloBoolVar>> var_x(env,n);
	IloArray<IloIntVar> var_y(env,n);

	for(int i=0;i<n;i++){
		var_x[i]=IloArray<IloBoolVar>(env,n);
		ostringstream stmy;
		stmy<<"y["<<i<<"]";
		var_y[i]=IloIntVar(env,stmy.str().c_str());
		for(int j=0;j<n;j++){
			ostringstream stm;
			stm<<"x"<<"["<<i<<"]["<<j<<"]";
			var_x[i][j]=IloBoolVar(env,stm.str().c_str());
		}
	}
	IloRangeArray constraint(env);
	for(int i=0;i<n;i++){
		IloExpr con1(env);
		for(int j=0;j<n;j++){
			if(j==i) continue;
			con1+=var_x[i][j];
		}
		constraint.add(con1==1);
	}
	for(int j=0;j<n;j++){
		IloExpr con2(env);
		for(int i=0;i<n;i++){
			if(j==i) continue;
			con2+=var_x[i][j];
		}
		constraint.add(con2==1);
	}
	for(int i=0;i<n;i++){
		for(int j=0;j<n;j++){
			if(j==i || i==0 || j==0) continue;
			IloExpr con3(env);
			con3+=n*var_x[i][j];
			con3+=var_y[i];
			con3-=var_y[j];
			constraint.add(con3<=n-1);
		}
	}
	IloExpr objective(env);
	for(int i=0;i<n;i++)
		for(int j=0;j<n;j++){
			if(i==j) continue;
			objective+=dist(points[i],points[j])*var_x[i][j];
		}


	IloModel model(env);
	model.add(constraint);
	model.add(IloMinimize(env,objective));
	IloCplex cplex(model);
	solveLP(cplex,2);

	
	of<<"Solution status = " << cplex.getStatus()<<endl;
	of<<"Objective function value = "<<cplex.getObjValue()<<endl;
	of<<"Optimal solutions = "<<endl;
	for(int k=0;k<n;k++){
		for(int j=0;j<n;j++){
			if(k==j) continue;
			int val=cplex.getValue(var_x[k][j]);
			if(val==1) of<<"x["<<k<<"]["<<j<<"] = "<<val<<" ";
		}
		of<<endl;
	}
	for(int k=1;k<n;k++){
		of<<"u["<<k<<"] = "<<cplex.getValue(var_y[k])<<" ";
	}
	env.end();
}
void construct_model3_1(){
	IloEnv env;
	IloArray<IloArray<IloBoolVar>> var_x(env,n);
	IloArray<IloArray<IloArray<IloBoolVar>>> var_y(env,n);
	
	for(int i=0;i<n;i++){
		var_y[i]=IloArray<IloArray<IloBoolVar>>(env,n);
		for(int j=0;j<n;j++){
			var_y[i][j]=IloArray<IloBoolVar>(env,n);
			for(int k=0;k<n;k++){
				ostringstream stm;
				stm<<"y"<<"["<<i<<"]["<<j<<"]["<<k<<"]";
				var_y[i][j][k]=IloBoolVar(env,stm.str().c_str());
			}
		}
	}
	for(int i=0;i<n;i++){
		var_x[i]=IloArray<IloBoolVar>(env,n);
		for(int j=0;j<n;j++){
			ostringstream stm;
			stm<<"x"<<"["<<i<<"]["<<j<<"]";
			var_x[i][j]=IloBoolVar(env,stm.str().c_str());
		}
	}
	IloRangeArray constraint(env);
	for(int i=0;i<n;i++){
		IloExpr con1(env);
		for(int j=0;j<n;j++){
			con1+=var_x[i][j];
		}
		constraint.add(con1==1);
	}
	for(int j=0;j<n;j++){
		IloExpr con2(env);
		for(int i=0;i<n;i++){
			con2+=var_x[i][j];
		}
		constraint.add(con2==1);
	}
	for(int k=0;k<n;k++){
		for(int i=0;i<n;i++){
			for(int j=0;j<n;j++){
				IloExpr conl(env);
				IloExpr conr(env);
				IloExpr con(env);
				conl+=2*var_y[k][i][j]-var_x[i][k]-var_x[j][(k+1)%n];
				con+=var_x[i][k]+var_x[j][(k+1)%n]-var_y[k][i][j];
				constraint.add(conl<=0);
				constraint.add(con<=1);
			}
		}
	}
		
	IloExpr objective(env);
	for(int i=0;i<n;i++)
		for(int j=0;j<n;j++)
			for(int k=0;k<n;k++)
				objective+=dist(points[i],points[j])*var_y[k][i][j];

	IloModel model(env);
	model.add(constraint);
	model.add(IloMinimize(env,objective));
	IloCplex cplex(model);
	solveLP(cplex,3);

		
	of<<"Solution status = " << cplex.getStatus()<<endl;
	of<<"Objective function value = "<<cplex.getObjValue()<<endl;
	of<<"Optimal solutions = "<<endl;
	for(int i=0;i<n;i++){
		for(int j=0;j<n;j++){
			for(int k=0;k<n;k++){
				int val=cplex.getValue(var_y[i][j][k]);
				if(val==1) of<<"y["<<i<<"]["<<j<<"]["<<k<<"] = "<<val<<" ";
			}
		}
		of<<endl;
	}
	for(int k=0;k<n;k++){
		for(int j=0;j<n;j++){
			int val=cplex.getValue(var_x[k][j]);
			if(val==1) of<<"x["<<k<<"]["<<j<<"] = "<<val<<" ";
		}
		of<<endl;
	}
	env.end();
}
void construct_model3_2(){
	IloEnv env;
	IloArray<IloArray<IloArray<IloBoolVar>>> var_y(env,n);
	
	for(int i=0;i<n;i++){
		var_y[i]=IloArray<IloArray<IloBoolVar>>(env,n);
		for(int j=0;j<n;j++){
			var_y[i][j]=IloArray<IloBoolVar>(env,n);
			for(int k=0;k<n;k++){
				ostringstream stm;
				stm<<"y"<<"["<<i<<"]["<<j<<"]["<<k<<"]";
				var_y[i][j][k]=IloBoolVar(env,stm.str().c_str());
			}
		}
	}
	
	IloRangeArray constraint(env);
	for(int k=0;k<n;k++){
		IloExpr con1(env);
		IloExpr con2(env);
		IloExpr con3(env);
		for(int i=0;i<n;i++)
			for(int j=0;j<n;j++){
				if(i!=j) con1+=var_y[k][i][j];
				if(k==j) continue;
				con2+=var_y[i][k][j];
				con3+=var_y[i][j][k];
			}
		constraint.add(con1==1);
		constraint.add(con2==1);
		constraint.add(con3==1);
	}
	
	for(int k=0;k<n;k++){
		for(int j=0;j<n;j++){
			IloExpr con(env);
			for(int i=0;i<n;i++){
				if(i!=j) con+=var_y[k][i][j]-var_y[(k+1)%n][j][i];
			}
			constraint.add(con==0);
		}
	}
		
	IloExpr objective(env);
	for(int k=0;k<n;k++)
		for(int i=0;i<n;i++)
			for(int j=0;j<n;j++)
				if(i!=j) objective+=dist(points[i],points[j])*var_y[k][i][j];

	IloModel model(env);
	model.add(constraint);
	model.add(IloMinimize(env,objective));
	IloCplex cplex(model);
	solveLP(cplex,4);

		
	of<<"Solution status = " << cplex.getStatus()<<endl;
	of<<"Objective function value = "<<cplex.getObjValue()<<endl;
	of<<"Optimal solutions = "<<endl;
	for(int i=0;i<n;i++){
		for(int j=0;j<n;j++){
			for(int k=0;k<n;k++){
				if(j==k) continue;
				int val=cplex.getValue(var_y[i][j][k]);
				if(val==1) of<<"y["<<i<<"]["<<j<<"]["<<k<<"] = "<<val<<" ";
			}
		}
		of<<endl;
	}
	env.end();
}
bool check_conn(IloEnv &env,IloCplex &cplex,IloArray<IloArray<IloBoolVar>> &var_x,IloRangeArray &constraint){
	bool *visit=new bool[n];
	int *root=new int[n];
	for(int i=0;i<n;i++){
		visit[i]=false;
		root[i]=i;
		for(int j=0;j<n;j++){
			mat[i][j]=false;
			if(i==j) continue;
			if(cplex.getValue(var_x[i][j])==1)
				mat[i][j]=true;
		}
	}
	int cur=0;
	while(1){
		visit[cur]=true;
		int walk=next(cur);
		while(walk!=cur){
			visit[walk]=true;
			root[walk]=cur;
			walk=next(walk);
		}
		cur=findf(visit);
		if(cur==0) break;
	}
	if(sum(root)==0) return true;
	IloExpr con(env);
	for(int i=0;i<n;i++){
		bool flag=false;
		IloExpr con(env);
		for(int j=0;j<n;j++)
			for(int k=0;k<n;k++)
				if(j!=k && root[j]==i && root[k]!=i){
					con+=var_x[j][k];
					flag=true;
				}
		if(flag) constraint.add(con>=1);
	}
	return false;
}

int next(int cur){
	for(int i=0;i<n;i++)
		if(i!=cur && mat[cur][i]) return i;
	return -1;
}
int findf(bool *visit){
	for(int i=0;i<n;i++)
		if(!visit[i]) return i;
	return 0;
}
int sum(int *root){
	int res=0;
	for(int i=0;i<n;i++)
		res+=root[i];
	return res;
}