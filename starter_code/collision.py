def aabb_intersect(p0, p1, aabb_min, aabb_max, eps=1e-8):
    """
    p0, p1, aabb_min, aabb_max: numpy.array of shape (3,)
    return True if collision。
    """
    dir = p1 - p0
    t_min, t_max = 0.0, 1.0

    for i in range(3):
        if abs(dir[i]) < eps:
            if p0[i] < aabb_min[i] or p0[i] > aabb_max[i]:
                return False
        else:
            inv = 1.0 / dir[i]
            t1 = (aabb_min[i] - p0[i]) * inv
            t2 = (aabb_max[i] - p0[i]) * inv
            t_enter = min(t1, t2)
            t_exit  = max(t1, t2)
            t_min = max(t_min, t_enter)
            t_max = min(t_max, t_exit)
            if t_min >= t_max:
                return False

    return True