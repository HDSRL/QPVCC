#ifndef QSQP_WRAPPER_H
#define QSQP_WRAPPER_H

#include "eigen3/Eigen/Dense"
#include "osqp/osqp.h"
#include "iostream"

/**
 * Create Compressed-Column-Sparse matrix from existing dense array
 * @param  m     First dimension
 * @param  n     Second dimension
 * @param  pr    Vector of data as standard array ( Eigen::Matrix<c_float,m,n>.data() )
 * @return       New matrix pointer
 */
inline csc *csc_matrix_dense(c_int m, c_int n, c_float *pr)
{
    c_int *Ap, *Ai;
    c_float *Ax;
    c_int cnt = 0;

    Ap = (c_int *)malloc((n + 1) * sizeof(c_int));
    for (c_int i = 0; i < n + 1; i++)
        Ap[i] = 0;

    for (c_int j = 0; j < n; j++){
        for (c_int i = 0; i < m; i++){
            if (pr[j * m + i] != 0.0)
                cnt++;
        }
        Ap[j + 1] = cnt;
    }

    Ai = (c_int *)malloc((cnt) * sizeof(c_int));
    Ax = (c_float *)malloc((cnt) * sizeof(c_float));
    cnt = 0;
    for (c_int j = 0; j < n; j++){
        for (c_int i = 0; i < m; i++){
            if (pr[j * m + i] != 0){
                Ai[cnt] = i;
                Ax[cnt] = pr[j * m + i];
                cnt++;
            }
        }
    }

    csc *M = (csc *)c_malloc(sizeof(csc));
    if (!M)
        return OSQP_NULL;

    M->m = m;
    M->n = n;
    M->nz = -1;
    M->nzmax = cnt;
    M->x = Ax;
    M->i = Ai;
    M->p = Ap;
    return M;
}

/**
 * Create Compressed-Column-Sparse matrix from existing eigen matrix
 * @param  X     Eigen matrix (expected to be of type c_float)
 * @return       New matrix pointer
 */
template <typename T>
inline csc *csc_matrix_eig(Eigen::DenseBase<T> &X)
{
    c_int m = X.rows();
    c_int n = X.cols();
    
    return csc_matrix_dense(m,n,X.derived().data());
}

/**
 * Frees the data in the Compressed-Column-Sparse matrix created by csc_matrix_eig
 * @param  X     pointer to csc data structure
 * @return       void
 */
inline void freeCSC(csc *X)
{
    c_free(X->x);
    c_free(X->i);
    c_free(X->p);
    c_free(X);
}

/**
 * Properly frees the OSQPData structure
 * @param  data  pointer to OSQPData structure
 * @return       void
 */
inline void freeData(OSQPData *data){
    if (data) {
        if (data->A) freeCSC(data->A);
        if (data->P) freeCSC(data->P);
        c_free(data);
    }
}


#endif