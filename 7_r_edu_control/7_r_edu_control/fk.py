import numpy as np
import math
import modern_robotics as mr

class Forward_kinamatics:
    def __init__(self,w,q,M,theta):
     
        self.w=w
        self.v=np.cross(q,w)
        self.M=M
        self.theta=theta
        self.S_list=np.hstack((self.w, self.v)).T
        
        print(mr.FKinSpace(self.M,self.S_list,self.theta))
        
    def calculate(self,num_of_joints):
        # step 1 Converts a spatial velocity vector[S] into a 4x4 matrix in se3 [w v , 0 1]
        w_hat = np.zeros((4, 4))
        T=self.M
        for i in range(num_of_joints-1,-1,-1):
            self.updated_s_list=  self.S_list[:,i]*self.theta[i] #[0. 0. 0. 0. 0. 0.]
            
            w_theta=np.array([self.updated_s_list[0], self.updated_s_list[1], self.updated_s_list[2]]) # vector representing the angular velocity.

            w_hat[:3, :3] = np.array([[0,-w_theta[2],w_theta[1]],[w_theta[2],0,-w_theta[0]],[-w_theta[1],w_theta[0],0]])

            w_hat[0,3]=self.updated_s_list[3]
            w_hat[1,3]=self.updated_s_list[4]
            w_hat[2,3]=self.updated_s_list[5]
            # print(w_hat)
        # step 2 """Computes the matrix exponential of an se3 representation of exponential coordinates
            # print(w_hat,i,w_theta)
            if abs(np.linalg.norm(w_theta))<1e-6:
                
                SE3= np.vstack([np.hstack([np.eye(3), self.updated_s_list[3:6].reshape(-1, 1)]), np.array([0, 0, 0, 1])])
            else:
                theta=np.linalg.norm(w_theta)
                omgmat = w_hat[0:3,0:3] / theta
                # Computes the matrix exponential of a matrix in so(3)
                if abs(np.linalg.norm(w_theta))<1e-6:
                    MatrixExp3= np.eye(3)
                else:
                    MatrixExp3= np.eye(3) + np.sin(theta) * omgmat + (1 - np.cos(theta)) * np.dot(omgmat, omgmat)

                translation = np.dot(np.eye(3) * theta + (1 - np.cos(theta)) * omgmat + (theta - np.sin(theta)) * np.dot(omgmat, omgmat), w_hat[0:3, 3]) / theta

                SE3 = np.vstack([np.hstack([MatrixExp3, translation.reshape(-1, 1)]), [0, 0, 0, 1]])
                T = np.dot(SE3,T)
            print('i',i,SE3)
        return T


if __name__ == "__main__":
    num_of_joints=7
    l1=0.15
    l2=0.24
    l3=0.43
    l4=0.43
    l5=0.23
    l6=0.02
    theta=np.array([0.527,-0.56,-0.39,-0.56,0,0,0])
    w=np.array([[0,0,1],[1,0,0],[0,0,1],[-1,0,0],[0,0,1],[1,0,0],[0,0,1]])
    q=np.array([[0,0,0],[l1,0,l2],[l1,0,l2+l3],[0,0,l2+l3],[0,0,l2+l3+l4],[l1,0,l2+l3+l4],[l1,0,l2+l3+l4+l5]])
    M=np.identity(4)
    M[0,3]=l1
    M[2,3]=l2+l3+l4+l5+l6
    fk=Forward_kinamatics(w,q,M,theta)
    hom_matrix=fk.calculate(num_of_joints)
    print(np.round(hom_matrix,4))