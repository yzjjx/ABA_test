#include <cmath>
#include <test_v_ori.h>
#include <test_p.h>

void tip_pass3(
    const data_t X_lam[DOF][6][6],
    const data_t c[DOF][6],
    const data_t D[DOF],
    const data_t u[DOF],
    const data_t U[DOF][6],
    data_t a_s[DOF][6],
    data_t a_a[DOF][6],
    data_t ddq[DOF]
)
{
    data_t S[6] = {0, 0, 1, 0, 0, 0};
    data_t a_parent[6] = {0, 0, 0, 0, 0, 0};

    for(int i = 0;i<DOF;i++)
    {
        for(int j = 0;j<6;j++)
        {
            a_s[i][j] =  X_lam[i][j][0]*a_parent[0]+X_lam[i][j][1]*a_parent[1]
                        +X_lam[i][j][2]*a_parent[2]+X_lam[i][j][3]*a_parent[3]
                        +X_lam[i][j][4]*a_parent[4]+X_lam[i][j][5]*a_parent[5]
                        + c[i][j];
        }
        
        ddq[i] = (u[i]-(U[i][0]*a_s[i][0]+U[i][1]*a_s[i][1]
                  +U[i][2]*a_s[i][2]+U[i][3]*a_s[i][3]
                  +U[i][4]*a_s[i][4]+U[i][5]*a_s[i][5]))/D[i];
                  
        for(int k = 0;k<6;k++)
        {
            a_a[i][k] = a_s[i][k] + S[k]*ddq[i];
            a_parent[k] = a_a[i][k];
        }
    }
}