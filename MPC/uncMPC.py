import numpy as np
from scipy.linalg import block_diag

def uncMPC(N, A, B, Q, R, P):
    n = B.shape[0]
    m = B.shape[1]

    # Build S
    S = np.zeros((N * n, N * m))
    S[:n, :m] = B
    row = 0
    prev = B
    
    for i in range(N - 1):
        next = np.hstack((prev, A.dot(prev[:, -m:])))
        ncols = next.shape[1]
        row += m
        S[row:row + m, :ncols] = next
        prev = next

    # Build M
    M = np.zeros((N * n, n))
    k = 0
    for i in range(N):
        M[k:k + n, :] = np.linalg.matrix_power(A, i)
        k += n

    # Build Qbar
    QQ = np.tile(Q, (N, 1))
    Qcell = np.split(QQ, N)
    Qbar = block_diag(*Qcell)
    Qbar[-n:, -n:] = P

    # Build Rbar
    RR = np.tile(R, (N, 1))
    Rcell = np.split(RR, N)
    Rbar = block_diag(*Rcell)

    # Compute K0N
    premult = np.zeros((m, m * N))
    premult[:m, :m] = -np.eye(m)
    temp1 = S.T.dot(Qbar).dot(S) + Rbar
    temp2 = np.linalg.inv(temp1)
    K0N = premult.dot(temp2).dot(S.T).dot(Qbar).dot(M) 

    return S, M, Qbar, Rbar, K0N
