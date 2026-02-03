#include<bits/stdc++.h>
using namespace std;


const int N = 6;

void derivatives(double x[], double xdot[])
{
    // First second-order system
    xdot[0] = x[1];
    xdot[1] = -x[0] - 0.5 * x[1];

    // Second second-order system
    xdot[2] = x[3];
    xdot[3] = -2.0 * x[2] - 0.3 * x[3];

    // Third second-order system
    xdot[4] = x[5];
    xdot[5] = -0.8 * x[4] - 0.2 * x[5];
}

void rk4_step(double x[], double dt)
{
    double k1[N], k2[N], k3[N], k4[N];
    double xtemp[N];

    // k1
    derivatives(x, k1);

    // k2
    for (int i = 0; i < N; i++)
        xtemp[i] = x[i] + 0.5 * dt * k1[i];
    derivatives(xtemp, k2);

    // k3
    for (int i = 0; i < N; i++)
        xtemp[i] = x[i] + 0.5 * dt * k2[i];
    derivatives(xtemp, k3);

    // k4
    for (int i = 0; i < N; i++)
        xtemp[i] = x[i] + dt * k3[i];
    derivatives(xtemp, k4);

    // Final update
    for (int i = 0; i < N; i++)
    {
        x[i] += (dt / 6.0) *
                (k1[i] + 2.0 * k2[i] + 2.0 * k3[i] + k4[i]);
    }
}

int main()
{
    double x[N] = {
        1.0, 0.0,   // y1, y1_dot
        0.5, 0.0,   // y2, y2_dot
        0.2, 0.0    // y3, y3_dot
    };

    double dt = 0.01;
    int steps = 2000;

    cout << "t\t"
         << "x1\t x2\t x3\t x4\t x5\t x6\n";

    for (int i = 0; i < steps; i++)
    {
        double t = i * dt;

        cout << t << "\t";
        for (int j = 0; j < N; j++)
            cout << x[j] << "\t";
        cout << endl;

        rk4_step(x, dt);
    }

    return 0;
}
