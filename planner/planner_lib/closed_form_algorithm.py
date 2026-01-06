import numpy as np

def closed_form_algorithm(goal_matrix, q_current, type):
    ## Robot data: UR5
    
    # Link length
    # L1, L2, L3, L4 = 0.0892, 0.1359, 0.4250, 0.1197
    # L5, L6, L7, L8 = 0.3923, 0.0930, 0.0947, 0.0823
    
    # # d
    # vd1 = L1
    # vd2 = 0
    # vd3 = 0
    # vd4 = L2 - L4 + L6
    # vd5 = L7
    # vd6 = L8
    
    # # a
    # va1 = 0
    # va2 = -L3
    # va3 = -L5
    # va4 = 0
    # va5 = 0
    # va6 = 0

    ## Robot data: UR10

    # Link length
    # L1, L2, L3, L4 = 
    # L5, L6, L7, L8 = 

    # d
    vd1 = 0.1807
    vd2 = 0
    vd3 = 0
    vd4 = 0.17415
    vd5 = 0.11985
    vd6 = 0.11655
    
    # a
    va1 = 0
    va2 = -0.6127
    va3 = -0.57155
    va4 = 0
    va5 = 0
    va6 = 0
    
    # Previous determinations
    px, py, pz = goal_matrix[0, 3], goal_matrix[1, 3], goal_matrix[2, 3]
    r11, r12, r13 = goal_matrix[0, 0], goal_matrix[0, 1], goal_matrix[0, 2]
    r21, r22, r23 = goal_matrix[1, 0], goal_matrix[1, 1], goal_matrix[1, 2]
    r31, r32, r33 = goal_matrix[2, 0], goal_matrix[2, 1], goal_matrix[2, 2]

    ###############################################
    ### Closed form Algorithm 1 (All solutions) ###
    ###############################################

    if type == 0:
        sol = np.full((8, 6), np.nan)   # rows: number of solutions, cols: joint values

        ### Step 1 - Both q1 solutions are computed, and complex angles are discarded.
        A = py - vd6 * r23
        B = px - vd6 * r13
        q1_vals = []    # fnal size = 2 (number of diferent values)

        # Checking valid values of q1 (result inside sqrt != imginary)
        try:
            q1_1 = np.arctan2(np.sqrt(B**2 + A**2 - vd4**2), vd4) + np.arctan2(B, -A)
            q1_vals.append(q1_1)
            sol[0:4, 0] = q1_1
        except:
            q1_vals.append(np.nan)
        try:
            q1_2 = -np.arctan2(np.sqrt(B**2 + A**2 - vd4**2), vd4) + np.arctan2(B, -A)
            q1_vals.append(q1_2)
            sol[4:8, 0] = q1_2
        except:
            q1_vals.append(np.nan)

        ### Step 2 - Compute q5. The sets containing values of q5 that are not considered valid are rejected.
        q5_vals = []    # final size = 4 (number of diferent values)
        for i, q1_i in enumerate(q1_vals):
            if np.isnan(q1_i): q5_vals += [np.nan, np.nan]; continue
            C = np.sin(q1_i) * r11 - np.cos(q1_i) * r21
            D = np.cos(q1_i) * r22 - np.sin(q1_i) * r12
            # Checking valid values of q5 (real and |s5|>1e-12)
            try:
                s5 = np.sin(q1_i) * r13 - np.cos(q1_i) * r23
                q5_1 = np.arctan2(np.sqrt(C**2 + D**2), s5)
                if np.isreal(q5_1) and abs(np.sin(q5_1)) > 1e-12:
                    q5_vals.append(q5_1)
                    sol[i * 4 + 0, 4] = q5_1
                    sol[i * 4 + 1, 4] = q5_1
                else:
                    q5_vals.append(np.nan)
                q5_2 = -np.arctan2(np.sqrt(C**2 + D**2), s5)
                if np.isreal(q5_2) and abs(np.sin(q5_2)) > 1e-12:
                    q5_vals.append(q5_2)
                    sol[i * 4 + 2, 4] = q5_2
                    sol[i * 4 + 3, 4] = q5_2
                else:
                    q5_vals.append(np.nan)
            except:
                q5_vals += [np.nan, np.nan]

        ### Step 3 - q6 is computed for the remaining sets.
        q6_vals = []    # final size = 4 (number of diferent values)
        for i in range(4):
            q1_i, q5_i = sol[i * 2, 0], sol[i * 2, 4]
            if np.isnan(q5_i): q6_vals.append(np.nan); continue
            C = np.sin(q1_i) * r11 - np.cos(q1_i) * r21
            D = np.cos(q1_i) * r22 - np.sin(q1_i) * r12
            q6_i = np.arctan2(D / np.sin(q5_i), C / np.sin(q5_i))
            q6_vals.append(q6_i)
            sol[i * 2, 5] = q6_i
            sol[i * 2 + 1, 5] = q6_i


        ### Step 4 - q3 computed and verified. Again, the solutions with angles that are not acceptable are discarded.
        qaux, PC, PS = [], [], []
        for i in range(4):
            q1_i = sol[i * 2, 0]
            q5_i = sol[i * 2, 4]
            q6_i = sol[i * 2, 5]
            if np.isnan(q6_i): continue
            E = np.cos(q1_i) * r11 + np.sin(q1_i) * r21
            F = np.cos(q5_i) * np.cos(q6_i)
            qaux_i = np.arctan2(r31 * F - np.sin(q6_i) * E, F * E + np.sin(q6_i) * r31)
            PC_i = np.cos(q1_i) * px + np.sin(q1_i) * py - np.sin(qaux_i) * vd5 + np.cos(qaux_i) * np.sin(q5_i) * vd6
            PS_i = pz - vd1 + np.cos(qaux_i) * vd5 + np.sin(qaux_i) * np.sin(q5_i) * vd6
            qaux.extend([qaux_i, qaux_i])
            PC.extend([PC_i, PC_i])
            PS.extend([PS_i, PS_i])
            # Checking valid values of q3 (real and |s3|>1e-12)
            try:
                cosval = (PS_i**2 + PC_i**2 - va2**2 - va3**2) / (2 * va2 * va3)
                if (1 - cosval**2) >= 0:
                    q3_1 = np.arctan2(np.sqrt(1 - cosval**2), cosval)
                    q3_2 = -q3_1
                    if abs(np.sin(q3_1)) > 1e-12:
                        sol[i * 2, 2] = q3_1
                    if abs(np.sin(q3_2)) > 1e-12:
                        sol[i * 2 + 1, 2] = q3_2
                # else:
                    # print('q3 not real')
            except:
                # print('q3 not computed')
                continue            

        ### Step 5 - q2 and q4 computed, and the sets of angles that are not valid are rejected.
        for i in range(8):
            q3_i = sol[i, 2]
            if np.isnan(q3_i): continue
            qaux_i, PC_i, PS_i = qaux[i], PC[i], PS[i]
            q2_i = np.arctan2(PS_i, PC_i) - np.arctan2(np.sin(q3_i) * va3, np.cos(q3_i) * va3 + va2)
            q4_i = qaux_i - q2_i - q3_i
            condition = vd5 * np.sin(qaux_i) + va2 * np.cos(q2_i) + va3 * np.cos(q2_i + q3_i)
            if abs(condition) > 1e-9:
                sol[i, 1] = q2_i
                sol[i, 3] = q4_i

        ### Step 6 - Solution with the minimal difference with respect to the current joint positions.
        # print("Solutions found are: \n",sol)
        weights = np.ones(6)
        # weights = []
        
        def normalize_angle_pi(angle):
            return (angle + np.pi) % (2 * np.pi) - np.pi
        
        def normalize_angle_2pi(angle):
            # return (angle + np.pi) % (2 * np.pi) - np.pi
            return (angle + 2*np.pi) % (4 * np.pi) - 2 * np.pi
        
        def unwrap_angles(q_new, q_ref):
            q_new = np.asarray(q_new).flatten()
            q_ref = np.asarray(q_ref).flatten()
            q_unwrapped = np.zeros_like(q_new)
            for i in range(len(q_new)):
                delta = q_new[i] - q_ref[i]
                if delta > np.pi:
                    q_unwrapped[i] = q_new[i] - 2 * np.pi
                elif delta < -np.pi:
                    q_unwrapped[i] = q_new[i] + 2 * np.pi
                else:
                    q_unwrapped[i] = q_new[i]
            return q_unwrapped

        # print('All solutions: ',sol)
        valid_rows = ~np.isnan(sol).any(axis=1)  # Valid row if there is no nan value
        if np.any(valid_rows):
            diffs = np.array([
                np.sqrt(np.sum(weights * np.abs(np.arctan2(np.sin(q_current - sol[i]), np.cos(q_current - sol[i])))))
                for i in range(8) if valid_rows[i]
            ])
            idx_valid = np.where(valid_rows)[0]
            idx = idx_valid[np.argmin(diffs)]
            best_sol = sol[idx]
            best_sol = unwrap_angles(best_sol, q_current)
            best_sol = np.array([normalize_angle_2pi(angle) for angle in best_sol])
            # print('Best solution: ',best_sol)
            return best_sol
        else:
            print('No feasible solution found!')
            return np.full(6, np.nan)  # Not valid solutions found

    #####################################
    ### Closed form Algorithm 2 (FSM) ###
    #####################################

    elif type == 1:
        # FSM States
        S1, S5, S6, S3, S24, Send = range(6)
        current_state = S1

        # Initializations
        q1, q2, q3, q4, q5, q6 = [], [], [], [], [], []     # joint variables for the computed solutions
        ch1 = ch3 = ch5 = 0     # variables to indicate if the angles have been changed from the initial selection
        Z = []                  # complete solution set
        v_end = 0

        if current_state == S1:     # Case S1: q1 computed and verified
            A = py - vd6 * r23
            B = px - vd6 * r13
            try:
                q1_1 = np.arctan2(np.sqrt(B**2 + A**2 - vd4**2), vd4) + np.arctan2(B, -A)
                q1.append(q1_1)
            except:
                pass
            try:
                q1_2 = -np.arctan2(np.sqrt(B**2 + A**2 - vd4**2), vd4) + np.arctan2(B, -A)
                q1.append(q1_2)
            except:
                pass
            if not q1:          # q1 not ok
                    Z = []
                    current_state = Send
            elif len(q1) == 2:  # q1 ok (but 2 results ok)
                idx = np.argmin(np.abs(q_current[0] - np.array(q1)))
                q1 = [q1[idx], q1[1 - idx]]     # ordered list (w.r.t. distance to current q1)
                current_state = S5
            else:               # q1 ok (but only 1 result ok, same as size(q1,2) == 1)
                current_state = S5

        elif current_state == S5:   # Case S5: q5 computed and verified
            q1_i = q1[0]
            C = np.sin(q1_i) * r11 - np.cos(q1_i) * r21
            D = np.cos(q1_i) * r22 - np.sin(q1_i) * r12
            s5 = np.sin(q1_i) * r13 - np.cos(q1_i) * r23
            try:
                q5_1 = np.arctan2(np.sqrt(C**2 + D**2), s5)
                q5_2 = -q5_1
                q5_cands = [q5_1, q5_2]
                q5 = [q for q in q5_cands if np.isreal(q) and abs(np.sin(q)) > 1e-12]
                if len(q5) == 2:    # q5 ok
                    idx = np.argmin(np.abs(q_current[4] - np.array(q5)))
                    q5 = [q5[idx], q5[1 - idx]]     # ordered list (w.r.t. distance to current q5)
                    current_state = S6
                else:
                    raise ValueError
            except:
                if ch1 == 0 and len(q1) == 2:   # q5 not ok, ch1 = 0
                    ch1 = 1
                    q1 = [q1[1]]
                    current_state = S5
                else:       # q5 not ok, ch1 = 1
                    Z = []
                    current_state = Send

        elif current_state == S6:   # Case S6: q6 is computed for the remaining sets
            q1_i, q5_i = q1[0], q5[0]
            C = np.sin(q1_i) * r11 - np.cos(q1_i) * r21
            D = np.cos(q1_i) * r22 - np.sin(q1_i) * r12
            q6_i = np.arctan2(D / np.sin(q5_i), C / np.sin(q5_i))
            q6 = [q6_i]
            current_state = S3

        elif current_state == S3:   # Case S3: q3 computed and verified. Again, the solutions with angles that are not acceptable are discarded
            q1_i, q5_i, q6_i = q1[0], q5[0], q6[0]
            E = np.cos(q1_i) * r11 + np.sin(q1_i) * r21
            F = np.cos(q5_i) * np.cos(q6_i)
            qaux_i = np.arctan2(r31 * F - np.sin(q6_i) * E, F * E + np.sin(q6_i) * r31)
            PC_i = np.cos(q1_i) * px + np.sin(q1_i) * py - np.sin(qaux_i) * vd5 + np.cos(qaux_i) * np.sin(q5_i) * vd6
            PS_i = pz - vd1 + np.cos(qaux_i) * vd5 + np.sin(qaux_i) * np.sin(q5_i) * vd6
            try:
                cosval = (PS_i**2 + PC_i**2 - va2**2 - va3**2) / (2 * va2 * va3)
                q3_1 = np.arctan2(np.sqrt(1 - cosval**2), cosval)
                q3_2 = -q3_1
                q3_cands = [q3_1, q3_2]
                q3 = [q for q in q3_cands if np.isreal(q) and abs(np.sin(q)) > 1e-12]
                if q3:      # q3 ok
                    idx = np.argmin(np.abs(q_current[2] - np.array(q3)))
                    q3 = [q3[idx], q3[1 - idx]]
                    current_state = S24
                else:
                    raise ValueError
            except:
                if ch5 == 0:    # q3 not ok, ch5 = 0
                    ch5 = 1
                    q5 = [q5[1]]
                    current_state = S6
                elif ch1 == 0 and len(q1) == 2: # q3 not ok, ch5 = 1, ch1 = 0
                    ch1 = 1
                    ch5 = 0
                    q1 = [q1[1]]
                    q5 = []
                    current_state = S5
                else:       # q3 not ok, ch1 = 1, ch5 = 1
                    Z = []
                    current_state = Send

        
        elif current_state == S24:  # Case S24: q2 and q4 computed, and the sets of angles that are not valid are rejected
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
            if abs(condition) > 1e-9:   # q2,q4 ok
                Z = [q1_i, q2_i, q3_i, q4_i, q5_i, q6_i]
                current_state = Send
            else:
                if ch3 == 0:    # q2 q4 not ok, ch3 = 0
                    ch3 = 1
                    q3 = [q3[1]]
                    current_state = S24
                elif ch5 == 0:  # q2 q4 not ok, ch3 = 1, ch5 = 0
                    ch5 = 1
                    q5 = [q5[1]]
                    current_state = S6
                elif ch1 == 0 and len(q1) == 2: # q2 q4 not ok, ch3 = 1, ch5 = 1, ch1 = 0
                    ch1 = 1
                    ch5 = 0
                    q1 = [q1[1]]
                    q5 = []
                    current_state = S5
                else:       # q2 q4 not ok, ch1 = 1, ch3 = 1, ch5 = 1
                    Z = []
                    current_state = Send
        
        elif current_state == Send:
            v_end = 1
            return np.array(Z) if Z else np.full(6, np.nan)

    else:
        print("Error selecting algorithm!!")
        return np.full(6, np.nan)








