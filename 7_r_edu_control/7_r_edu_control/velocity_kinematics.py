import numpy as np
import math



class Velocity_kinamatics:
    def __init__(self,w,q,theta):    
        self.w=w
        self.v=np.cross(q,w)
        self.theta=theta
        self.S_list=np.hstack((self.w, self.v)).T
       

    def Jacobian(self,num_of_joints):
        # step 1 Converts a spatial velocity vector[S] into a 4x4 matrix in se3 [w v , 0 1]
        w_hat = np.zeros((4, 4))
        
        if num_of_joints ==1:
            return [self.S_list]
        else:
            T=np.eye(4)
            Jacobian = self.S_list[:, 0].reshape(6, 1)  # Ensure correct shape (6x1)

            for i in range(0, num_of_joints-1):  # Start from 1, as 0 is already added
                self.updated_s_list=  self.S_list[:,i]*self.theta[i] #[0. 0. 0. 0. 0. 0.]
            
                w_theta=np.array([self.updated_s_list[0], self.updated_s_list[1], self.updated_s_list[2]]) # vector representing the angular velocity.

                w_hat[:3, :3] = np.array([[0,-w_theta[2],w_theta[1]],[w_theta[2],0,-w_theta[0]],[-w_theta[1],w_theta[0],0]])

                w_hat[0,3]=self.updated_s_list[3]
                w_hat[1,3]=self.updated_s_list[4]
                w_hat[2,3]=self.updated_s_list[5]
                # print(w_hat)
            # step 2 """Computes the matrix exponential of an se3 representation of exponential coordinates
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
                T = np.dot(T, SE3)
                
                
                R, p = T[:3, :3], T[:3, 3]
                p_hat = np.array([[0, -p[2], p[1]], [p[2], 0, -p[0]], [-p[1], p[0], 0]])
                adjoint = np.vstack([np.hstack([R, np.zeros((3, 3))]), np.hstack([np.dot(p_hat, R), R])])

                new_column = np.dot(adjoint, self.S_list[:, i+1]).reshape(6, 1)  # Reshape to (6x1)
                Jacobian = np.hstack((Jacobian, new_column))  # Stack horizontally

            return Jacobian

        


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
    
    vk=Velocity_kinamatics(w,q,theta)
    hom_matrix=vk.Jacobian(num_of_joints)

    print(np.round(hom_matrix,2))