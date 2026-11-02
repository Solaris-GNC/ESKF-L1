import numpy as np

def skew(x:np.ndarray) -> np.ndarray:
    """Skew-symmetric matrix [x]× such that [x]× y = x × y (eq. 33-34).

    Args:
        x: Vector, shape (3,).

    Returns:
        [x]×, shape (3, 3).
    """
    R1 = [0, -x[2], x[1]]
    R2 = [x[2], 0, -x[0]]
    R3 = [-x[1], x[0], 0]
    return np.array([R1, R2, R3])
