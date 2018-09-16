#include <iostream>
#include <fstream>
#include <vector>

using namespace std;

void VerletIntegration(){
    // declare variables
    double m, k, x, v, t_max, dt, t, a, x_mid;
    vector<double> t_list, x_list, v_list, f_list;
    
    // mass, spring constant, initial position and velocity
    m = 1;
    k = 1;
    x = 1;
    v = 1;
    
    // simulation time and timestep
    t_max = 1000;
    dt = 1.99;
    int index;
    // Verlet integration
    /*
     % verlet algorithm
     r(1) = r0;    v(1) = v0;    F(1) = -k*r(1);
     r(2) = r(1)+v(1)*dt+0.5*dt*dt*F(1)/m;
     F(2) = -k*r(2);
     
     for i = 2:totstep
     r(i+1) = 2*r(i)-r(i-1)+dt*dt*F(i)/m;
     v(i) = 0.5*(r(i+1)-r(i-1))/dt;
     F(i+1) = -k*r(i+1);
     end
     */
    t=0;
    x_list.push_back(x);
    v_list.push_back(v);
    f_list.push_back(-k*x);
    x_list.push_back(x_list[0]+v_list[0]+0.5*dt*dt*f_list[0]/m);
    f_list.push_back(-k*x_list[1]);
    t_list.push_back(t);
    t=t+dt;
    t_list.push_back(t);
    t=t+dt;
    cout<<x_list[0]<<endl<<x_list[1]<<endl;
    
    
    for (t=t; t <= t_max; t = t + dt) {
        index=x_list.size()-1;
        x_list.push_back(2*x_list[index] - x_list[index-1] + dt*dt*f_list[index]/m);
        v_list.push_back(0.5*(x_list[index+1] - x_list[index-1])/dt);
        f_list.push_back(-k*x_list[index+1]);
        t_list.push_back(t);
    }
    cout<<index;
    v_list.push_back(v_list[index]);
    
    
    cout<<"running....\n";
    // Write the trajectories to file
    ofstream fout;
    fout.open("/Users/dvmsc/百度云同步盘/电脑同步/Program/C++/lander/lander/spring/trajectories.txt");
    if (fout) { // file opened successfully
        for (int i = 0; i < t_list.size(); i = i + 1) {
            fout << t_list[i] << ' ' << x_list[i] << ' ' << v_list[i] << endl;
        }
    } else { // file did not open successfully
        cout << "Could not open trajectory file for writing" << endl;
    }
}

void EulerIntegration(){
    // declare variables
    double m, k, x, v, t_max, dt, t, a;
    vector<double> t_list, x_list, v_list;
    
    // mass, spring constant, initial position and velocity
    m = 1;
    k = 1;
    x = 0;
    v = 1;
    
    // simulation time and timestep
    t_max = 20;
    dt = 0.1;
    
    // Euler integration
    for (t = 0; t <= t_max; t = t + dt) {
        
        // append current state to trajectories
        t_list.push_back(t);
        x_list.push_back(x);
        v_list.push_back(v);
        cout<<x<<endl;
        // calculate new position and velocity
        a = -k * x / m;
        x = x + dt * v;
        v = v + dt * a;
        
    }
    
    cout<<"running....\n";
    // Write the trajectories to file
    ofstream fout;
    fout.open("/Users/dvmsc/百度云同步盘/电脑同步/Program/C++/lander/lander/spring/trajectories.txt");
    if (fout) { // file opened successfully
        for (int i = 0; i < t_list.size(); i = i + 1) {
            fout << t_list[i] << ' ' << x_list[i] << ' ' << v_list[i] << endl;
        }
    } else { // file did not open successfully
        cout << "Could not open trajectory file for writing" << endl;
    }

}
int main() {

    //EulerIntegration();
    VerletIntegration();
    /* The file can be loaded and visualised in Python as follows:

  import numpy as np
  import matplotlib.pyplot as plt
  results = np.loadtxt('trajectories.txt')
  plt.figure(1)
  plt.clf()
  plt.xlabel('time (s)')
  plt.grid()
  plt.plot(results[:, 0], results[:, 1], label='x (m)')
  plt.plot(results[:, 0], results[:, 2], label='v (m/s)')
  plt.legend()
  plt.show()

  */
}
