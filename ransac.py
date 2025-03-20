import numpy as np

class RANSAC:
    '''
    Find the homography matrix (matrix transformation) from image 1 to image 2
    ''' 
    def __init__(self, source, target, iteration=1000, threshold=50):
        '''
        source: array of keypoints of the image 1 
        target: array of keypoints of the image 2
        iteration: the number of loops 
        threshold: the error limit for inliers
        '''
        self.source = np.array(source)  
        self.target = np.array(target) 
        self.iteration = iteration
        self.threshold = threshold 
    
    def _computeHomography(self, source, target):
        '''
        Args: 
            source: selected keypoints of source image
            target: corresponding keypoints of target image
        Return:
            Homography matrix H (3x3)
        '''
        A = []
        for (x, y), (x_t, y_t) in zip(source, target):
            A.append([-x, -y, -1, 0, 0, 0, x*x_t, y*x_t, x_t])
            A.append([0, 0, 0, -x, -y, -1, x*y_t, y*y_t, y_t])
    
        A = np.array(A)
        U, S, Vt = np.linalg.svd(A)
        H = Vt[-1].reshape(3, 3)

        if H[2, 2] == 0:
            return np.linalg.inv(H)

        return H / H[2, 2]
        
    def ransacFindHomography(self):
        '''  
        Find the best homography matrix using RANSAC.
        '''
        bestH = None
        maxInliers = 0
        num_points = len(self.source)

        for _ in range(self.iteration):
            idx = np.random.choice(num_points, 4, replace=False)
            src = self.source[idx]
            tar = self.target[idx]
            H = self._computeHomography(src, tar)

            inliers = 0
            for i in range(num_points):
                x, y = self.source[i]
                x_t, y_t = self.target[i]

                projected = H @ np.array([x, y, 1])
                projected /= projected[2] 

                error = np.linalg.norm(np.array([x_t, y_t]) - projected[:2])
                if error < self.threshold:
                    inliers += 1

            if inliers > maxInliers:
                bestH = H
                maxInliers = inliers

        return bestH if bestH is not None else np.eye(3) 
