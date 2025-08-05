# src/utils.py

import sympy as sp


def skew(v: sp.Matrix) -> sp.Matrix:
    """
    Returns the skew-symmetric matrix (cross-product matrix) of a 3x1 vector.

    Args:
        v (sp.Matrix): 3x1 vector

    Returns:
        sp.Matrix: 3x3 skew-symmetric matrix
    """
    if v.shape != (3, 1):
        raise ValueError("Input must be a 3x1 vector.")

    return sp.Matrix([
        [0,      -v[2],  v[1]],
        [v[2],   0,     -v[0]],
        [-v[1],  v[0],   0]
    ])


def simplify_matrix(M: sp.Matrix) -> sp.Matrix:
    """
    Applies sympy.simplify() element-wise to a symbolic matrix.

    Args:
        M (sp.Matrix): symbolic matrix

    Returns:
        sp.Matrix: simplified matrix
    """
    return M.applyfunc(sp.simplify)


def pretty_print(expr: sp.Expr):
    """
    Nicely prints a symbolic expression or matrix using sympy.pretty().

    Args:
        expr (sp.Expr or sp.Matrix): expression to print
    """
    print(sp.pretty(expr, use_unicode=True))


def to_latex(expr: sp.Expr) -> str:
    """
    Converts a symbolic expression or matrix to LaTeX string.

    Args:
        expr (sp.Expr or sp.Matrix): expression to convert

    Returns:
        str: LaTeX code
    """
    return sp.latex(expr)
