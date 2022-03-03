function [J_diff_splitPoint, JDot_diff_nu_splitPoint] = compute_J_and_JDot_nu_splitPoint(obj, robot)
[JDiff_Lpoint,JDiff_Rpoint] = robot.get_spilitPoints_diff_jacobian();
[JDotNuDiff_Lpoint,JDotNuDiff_Rpoint] = robot.get_SpilitPoints_diff_JDot_nu();
J_diff_splitPoint = [JDiff_Lpoint ; JDiff_Rpoint];
JDot_diff_nu_splitPoint = [JDotNuDiff_Lpoint ; JDotNuDiff_Rpoint];
end