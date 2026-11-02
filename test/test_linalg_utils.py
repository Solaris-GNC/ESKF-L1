"""Tests for the skew-symmetric matrix."""
import numpy as np
import pytest

from linalg_utils import skew


def random_vectors(n=10, seed=0):
    """Random 3D vectors, reproducible from the seed."""
    rng = np.random.default_rng(seed)
    return [rng.normal(size=3) for _ in range(n)]


def test_skew_known():
    """[x]× of a known vector.

    [x]× = [[0, -x3, x2], [x3, 0, -x1], [-x2, x1, 0]] (eq. 33).
    """
    np.testing.assert_allclose(
        skew(np.array([1, 2, 3])),
        [[0, -3, 2], [3, 0, -1], [-2, 1, 0]],
    )


@pytest.mark.parametrize("a", random_vectors())
def test_skew_antisymmetric(a):
    """[x]×ᵀ = -[x]×.

    The cross product is anticommutative, a × b = -b × a, which is antisymmetry of [a]×.
    """
    np.testing.assert_allclose(skew(a).T, -skew(a))


@pytest.mark.parametrize("a", random_vectors())
def test_skew_is_cross_product(a):
    """[a]× b = a × b.

    The defining property (eq. 34): it turns the cross product into a matrix product, which
    is how it appears in Φ and G.
    """
    b = np.array([0.4, -1.5, 2.2])
    np.testing.assert_allclose(skew(a) @ b, np.cross(a, b), atol=1e-12)


@pytest.mark.parametrize("a", random_vectors())
def test_skew_self_cross_is_zero(a):
    """[a]× a = 0.

    a × a = 0 for any vector (parallel vectors), so a spans the kernel of [a]× and the matrix
    has rank 2: rotating a vector about itself does not change it.
    """
    np.testing.assert_allclose(skew(a) @ a, np.zeros(3), atol=1e-12)
