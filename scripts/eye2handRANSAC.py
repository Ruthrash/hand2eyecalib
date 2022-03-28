import random
import numpy as np
import baldor as br

class Eye2HandRANSAC:
    def __init__(self,As, Bs, solver, min_pts=3, iterations=1000, thresh=0.05) -> None:
        self.As = As 
        self.Bs = Bs 
        self.iterations = iterations 
        self.thresh = thresh 
        self.min_pts = min_pts
        # self.req_agreeable_pts = req_agreeable_pts
        self.inliers_idxs = list()
        self.solver = solver
        self.besterr = 60000
        self.bestlen = 0
        pass

    def compute_estimation_error(self, X, X_hat):
        #Rerror = np.eye(4)
        #Rerror[:3, :3] = np.dot(X[:3, :3].T, X_hat[:3, :3])
        rot_error = 0#abs(br.transform.to_axis_angle(Rerror)[1])
        trans_error = np.linalg.norm(X[:3, 3]-X_hat[:3, 3])
        return  trans_error, rot_error

    def Run(self,):
        for i in range(self.iterations):
            inliers = []
            index_list = range(len(self.As))
            maybe_inliers_idxs = random.sample(index_list, k=self.min_pts)
            A = [self.As[i] for i in maybe_inliers_idxs]
            B = [self.Bs[i] for i in maybe_inliers_idxs]
            X = self.solver(A,B)
            for idx in index_list:
                if idx not in maybe_inliers_idxs:
                    t_error, r_error = self.compute_estimation_error(np.matmul(self.As[idx],X),
                                                                    np.matmul(X,self.Bs[idx]))
                    if np.sum(t_error) < self.thresh:
                        maybe_inliers_idxs.append(idx)

            if len(maybe_inliers_idxs) > 10:
                inlierAs = [self.As[i] for i in maybe_inliers_idxs]
                inlierBs = [self.Bs[i] for i in maybe_inliers_idxs]
                better_X = self.solver(inlierAs, inlierBs)
                thiserror = 0 
                for maybe_inlier_idx in maybe_inliers_idxs:
                    t_error, r_error = self.compute_estimation_error(np.matmul(self.As[maybe_inlier_idx], better_X),
                                                                                np.matmul(better_X, self.Bs[maybe_inlier_idx]))
                    thiserror += np.abs(np.sum(t_error))

                thiserror = thiserror/len(maybe_inliers_idxs)
                if thiserror < self.besterr :
                    best_X = better_X
                    self.besterr = thiserror
                    self.bestlen = len(maybe_inliers_idxs)
        print('best', self.bestlen)
        return best_X



