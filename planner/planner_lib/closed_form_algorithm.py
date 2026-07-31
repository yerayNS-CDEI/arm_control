import numpy as np


def _ur10e_dh_params():
    """Return UR10e DH parameters used by the analytical IK."""
    vd1 = 0.1807
    vd2 = 0
    vd3 = 0
    vd4 = 0.17415
    vd5 = 0.11985
    vd6 = 0.11655

    va1 = 0
    va2 = -0.6127
    va3 = -0.57155
    va4 = 0
    va5 = 0
    va6 = 0

    return vd1, vd2, vd3, vd4, vd5, vd6, va1, va2, va3, va4, va5, va6


def closed_form_all_solutions(goal_matrix):
    """Compute all analytical IK solutions (up to 8 rows, NaN for invalid rows)."""
    vd1, vd2, vd3, vd4, vd5, vd6, va1, va2, va3, va4, va5, va6 = _ur10e_dh_params()

    px, py, pz = goal_matrix[0, 3], goal_matrix[1, 3], goal_matrix[2, 3]
    r11, r12, r13 = goal_matrix[0, 0], goal_matrix[0, 1], goal_matrix[0, 2]
    r21, r22, r23 = goal_matrix[1, 0], goal_matrix[1, 1], goal_matrix[1, 2]
    r31, r32, r33 = goal_matrix[2, 0], goal_matrix[2, 1], goal_matrix[2, 2]

    sol = np.full((8, 6), np.nan)

    # Step 1: q1 candidates
    A = py - vd6 * r23
    B = px - vd6 * r13
    q1_vals = []

    radicand_q1 = B**2 + A**2 - vd4**2
    if radicand_q1 >= 0:
        q1_1 = np.arctan2(np.sqrt(radicand_q1), vd4) + np.arctan2(B, -A)
        q1_vals.append(q1_1)
        sol[0:4, 0] = q1_1
    else:
        q1_vals.append(np.nan)

    if radicand_q1 >= 0:
        q1_2 = -np.arctan2(np.sqrt(radicand_q1), vd4) + np.arctan2(B, -A)
        q1_vals.append(q1_2)
        sol[4:8, 0] = q1_2
    else:
        q1_vals.append(np.nan)

    # Step 2: q5 candidates
    for i, q1_i in enumerate(q1_vals):
        if np.isnan(q1_i):
            continue
        C = np.sin(q1_i) * r11 - np.cos(q1_i) * r21
        D = np.cos(q1_i) * r22 - np.sin(q1_i) * r12
        s5 = np.sin(q1_i) * r13 - np.cos(q1_i) * r23

        q5_1 = np.arctan2(np.sqrt(C**2 + D**2), s5)
        if np.isreal(q5_1) and abs(np.sin(q5_1)) > 1e-12:
            sol[i * 4 + 0, 4] = q5_1
            sol[i * 4 + 1, 4] = q5_1

        q5_2 = -np.arctan2(np.sqrt(C**2 + D**2), s5)
        if np.isreal(q5_2) and abs(np.sin(q5_2)) > 1e-12:
            sol[i * 4 + 2, 4] = q5_2
            sol[i * 4 + 3, 4] = q5_2

    # Step 3: q6 candidates
    for i in range(4):
        q1_i, q5_i = sol[i * 2, 0], sol[i * 2, 4]
        if np.isnan(q5_i):
            continue
        C = np.sin(q1_i) * r11 - np.cos(q1_i) * r21
        D = np.cos(q1_i) * r22 - np.sin(q1_i) * r12
        q6_i = np.arctan2(D / np.sin(q5_i), C / np.sin(q5_i))
        sol[i * 2, 5] = q6_i
        sol[i * 2 + 1, 5] = q6_i

    # Step 4: q3 candidates
    qaux = [np.nan] * 8
    PC = [np.nan] * 8
    PS = [np.nan] * 8
    for i in range(4):
        row_even = i * 2
        row_odd = row_even + 1
        q1_i = sol[row_even, 0]
        q5_i = sol[row_even, 4]
        q6_i = sol[row_even, 5]
        if np.isnan(q6_i):
            continue

        E = np.cos(q1_i) * r11 + np.sin(q1_i) * r21
        F = np.cos(q5_i) * np.cos(q6_i)
        qaux_i = np.arctan2(r31 * F - np.sin(q6_i) * E, F * E + np.sin(q6_i) * r31)
        PC_i = np.cos(q1_i) * px + np.sin(q1_i) * py - np.sin(qaux_i) * vd5 + np.cos(qaux_i) * np.sin(q5_i) * vd6
        PS_i = pz - vd1 + np.cos(qaux_i) * vd5 + np.sin(qaux_i) * np.sin(q5_i) * vd6

        qaux[row_even] = qaux_i
        qaux[row_odd] = qaux_i
        PC[row_even] = PC_i
        PC[row_odd] = PC_i
        PS[row_even] = PS_i
        PS[row_odd] = PS_i

        cosval = (PS_i**2 + PC_i**2 - va2**2 - va3**2) / (2 * va2 * va3)
        if (1 - cosval**2) >= 0:
            q3_1 = np.arctan2(np.sqrt(1 - cosval**2), cosval)
            q3_2 = -q3_1
            if abs(np.sin(q3_1)) > 1e-12:
                sol[row_even, 2] = q3_1
            if abs(np.sin(q3_2)) > 1e-12:
                sol[row_odd, 2] = q3_2

    # Step 5: q2 and q4
    for i in range(8):
        q3_i = sol[i, 2]
        if np.isnan(q3_i) or np.isnan(qaux[i]) or np.isnan(PC[i]) or np.isnan(PS[i]):
            continue
        qaux_i, PC_i, PS_i = qaux[i], PC[i], PS[i]
        q2_i = np.arctan2(PS_i, PC_i) - np.arctan2(np.sin(q3_i) * va3, np.cos(q3_i) * va3 + va2)
        q4_i = qaux_i - q2_i - q3_i
        condition = vd5 * np.sin(qaux_i) + va2 * np.cos(q2_i) + va3 * np.cos(q2_i + q3_i)
        if abs(condition) > 1e-9:
            sol[i, 1] = q2_i
            sol[i, 3] = q4_i

    return sol


def closed_form_fsm_solution(goal_matrix, q_current):
    """Compute one IK solution using FSM branch exploration (legacy path)."""
    vd1, vd2, vd3, vd4, vd5, vd6, va1, va2, va3, va4, va5, va6 = _ur10e_dh_params()

    px, py, pz = goal_matrix[0, 3], goal_matrix[1, 3], goal_matrix[2, 3]
    r11, r12, r13 = goal_matrix[0, 0], goal_matrix[0, 1], goal_matrix[0, 2]
    r21, r22, r23 = goal_matrix[1, 0], goal_matrix[1, 1], goal_matrix[1, 2]
    r31, r32, r33 = goal_matrix[2, 0], goal_matrix[2, 1], goal_matrix[2, 2]

    S1, S5, S6, S3, S24, SEND = range(6)
    current_state = S1

    q1, q2, q3, q4, q5, q6 = [], [], [], [], [], []
    ch1 = ch3 = ch5 = 0
    Z = []

    while current_state != SEND:
        if current_state == S1:
            A = py - vd6 * r23
            B = px - vd6 * r13
            radicand_q1 = B**2 + A**2 - vd4**2
            if radicand_q1 >= 0:
                q1_1 = np.arctan2(np.sqrt(radicand_q1), vd4) + np.arctan2(B, -A)
                q1_2 = -np.arctan2(np.sqrt(radicand_q1), vd4) + np.arctan2(B, -A)
                q1 = [q1_1, q1_2]
            if not q1:
                Z = []
                current_state = SEND
            elif len(q1) == 2:
                idx = np.argmin(np.abs(q_current[0] - np.array(q1)))
                q1 = [q1[idx], q1[1 - idx]]
                current_state = S5
            else:
                current_state = S5

        elif current_state == S5:
            q1_i = q1[0]
            C = np.sin(q1_i) * r11 - np.cos(q1_i) * r21
            D = np.cos(q1_i) * r22 - np.sin(q1_i) * r12
            s5 = np.sin(q1_i) * r13 - np.cos(q1_i) * r23
            q5_1 = np.arctan2(np.sqrt(C**2 + D**2), s5)
            q5_2 = -q5_1
            q5_cands = [q5_1, q5_2]
            q5 = [q for q in q5_cands if np.isreal(q) and abs(np.sin(q)) > 1e-12]
            if len(q5) == 2:
                idx = np.argmin(np.abs(q_current[4] - np.array(q5)))
                q5 = [q5[idx], q5[1 - idx]]
                current_state = S6
            elif ch1 == 0 and len(q1) == 2:
                ch1 = 1
                q1 = [q1[1]]
                current_state = S5
            else:
                Z = []
                current_state = SEND

        elif current_state == S6:
            q1_i, q5_i = q1[0], q5[0]
            C = np.sin(q1_i) * r11 - np.cos(q1_i) * r21
            D = np.cos(q1_i) * r22 - np.sin(q1_i) * r12
            q6_i = np.arctan2(D / np.sin(q5_i), C / np.sin(q5_i))
            q6 = [q6_i]
            current_state = S3

        elif current_state == S3:
            q1_i, q5_i, q6_i = q1[0], q5[0], q6[0]
            E = np.cos(q1_i) * r11 + np.sin(q1_i) * r21
            F = np.cos(q5_i) * np.cos(q6_i)
            qaux_i = np.arctan2(r31 * F - np.sin(q6_i) * E, F * E + np.sin(q6_i) * r31)
            PC_i = np.cos(q1_i) * px + np.sin(q1_i) * py - np.sin(qaux_i) * vd5 + np.cos(qaux_i) * np.sin(q5_i) * vd6
            PS_i = pz - vd1 + np.cos(qaux_i) * vd5 + np.sin(qaux_i) * np.sin(q5_i) * vd6

            cosval = (PS_i**2 + PC_i**2 - va2**2 - va3**2) / (2 * va2 * va3)
            if (1 - cosval**2) >= 0:
                q3_1 = np.arctan2(np.sqrt(1 - cosval**2), cosval)
                q3_2 = -q3_1
                q3_cands = [q3_1, q3_2]
                q3 = [q for q in q3_cands if np.isreal(q) and abs(np.sin(q)) > 1e-12]
            else:
                q3 = []

            if q3:
                idx = np.argmin(np.abs(q_current[2] - np.array(q3)))
                q3 = [q3[idx], q3[1 - idx]]
                current_state = S24
            elif ch5 == 0:
                ch5 = 1
                q5 = [q5[1]]
                current_state = S6
            elif ch1 == 0 and len(q1) == 2:
                ch1 = 1
                ch5 = 0
                q1 = [q1[1]]
                q5 = []
                current_state = S5
            else:
                Z = []
                current_state = SEND

        elif current_state == S24:
            q3_i = q3[0]
            q1_i, q5_i, q6_i = q1[0], q5[0], q6[0]
            E = np.cos(q1_i) * r11 + np.sin(q1_i) * r21
            F = np.cos(q5_i) * np.cos(q6_i)
            qaux_i = np.arctan2(r31 * F - np.sin(q6_i) * E, F * E + np.sin(q6_i) * r31)
            PC_i = np.cos(q1_i) * px + np.sin(q1_i) * py - np.sin(qaux_i) * vd5 + np.cos(qaux_i) * np.sin(q5_i) * vd6
            PS_i = pz - vd1 + np.cos(qaux_i) * vd5 + np.sin(qaux_i) * np.sin(q5_i) * vd6
            q2_i = np.arctan2(PS_i, PC_i) - np.arctan2(np.sin(q3_i) * va3, np.cos(q3_i) * va3 + va2)
            q4_i = qaux_i - q2_i - q3_i
            condition = vd5 * np.sin(qaux_i) + va2 * np.cos(q2_i) + va3 * np.cos(q2_i + q3_i)
            if abs(condition) > 1e-9:
                Z = [q1_i, q2_i, q3_i, q4_i, q5_i, q6_i]
                current_state = SEND
            elif ch3 == 0:
                ch3 = 1
                q3 = [q3[1]]
                current_state = S24
            elif ch5 == 0:
                ch5 = 1
                q5 = [q5[1]]
                current_state = S6
            elif ch1 == 0 and len(q1) == 2:
                ch1 = 1
                ch5 = 0
                q1 = [q1[1]]
                q5 = []
                current_state = S5
            else:
                Z = []
                current_state = SEND

    return np.array(Z) if Z else np.full(6, np.nan)


def _select_best_solution(ik_solutions, q_ref=None):
    """Select one valid IK row, prioritizing proximity to q_ref when available."""
    valid_rows = ~np.isnan(ik_solutions).any(axis=1)
    if not np.any(valid_rows):
        return np.full(6, np.nan)

    valid_solutions = np.asarray(ik_solutions[valid_rows], dtype=float)
    if q_ref is None:
        return valid_solutions[0].copy()

    q_ref = np.asarray(q_ref, dtype=float)
    deltas = np.arctan2(np.sin(q_ref - valid_solutions), np.cos(q_ref - valid_solutions))
    distances = np.sqrt(np.sum(deltas ** 2, axis=1))
    best_sol = valid_solutions[int(np.argmin(distances))].copy()

    # Use the equivalent joint angles closest to q_ref.
    delta = (best_sol - q_ref + np.pi) % (2 * np.pi) - np.pi
    best_sol = q_ref + delta

    # Keep joints within UR limits.
    for i in range(len(best_sol)):
        while best_sol[i] > 2 * np.pi:
            best_sol[i] -= 2 * np.pi
        while best_sol[i] < -2 * np.pi:
            best_sol[i] += 2 * np.pi

    return best_sol


def closed_form_algorithm(goal_matrix, q_current=None, type=0, return_all_solutions=False):
    """
    Compatibility wrapper.

    type=0: by default return one best solution as a (6,) vector.
            if return_all_solutions=True, return all solutions as an (8, 6) matrix.
    type=1: return one FSM-based solution as a (6,) vector.
    """
    if type == 0:
        ik_solutions = closed_form_all_solutions(goal_matrix)
        if return_all_solutions:
            return ik_solutions
        return _select_best_solution(ik_solutions, q_current)
    if type == 1:
        if q_current is None:
            return np.full(6, np.nan)
        return closed_form_fsm_solution(goal_matrix, q_current)

    print("Error selecting algorithm!!")
    return np.full(6, np.nan)
