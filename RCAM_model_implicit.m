function F = RCAM_model_implicit(XDOT,X,U)
F = RCAM_model(X,U) - XDOT;