import numpy as np

class Pose2D:
    def __init__(self, x, y,theta):
    
        if not isinstance(x, (float, int)): 
            raise TypeError("x must be a float")
        self.x = x

        if not isinstance(y, (float, int)):
            raise TypeError("y must be a float")
        self.y = y  
        
        if not isinstance(theta, (float, int)): raise TypeError("theta must be a float") 


        self.theta = normalize_angle(theta)

        self.matrix = np.array([[np.cos(self.theta), -np.sin(self.theta), self.x],
                                         [np.sin(self.theta), np.cos(self.theta), self.y],[0,0,1]]) 

    def compose (self, other):
            """Compose deux poses : self = T_A<-B, other = T_B<-C, retour = T_A<-C.

            Exemple : pose_robot_monde.compose(pose_capteur_robot)
                    -> pose du capteur dans le monde.
            """
            if not isinstance (other, Pose2D) : raise TypeError("other must be a Pose2D")

            return matrix_to_pose(self.matrix @ other.matrix)

    def inverse(self):
        """Retourne la pose inverse : self = T_A<-B, retour = T_B<-A."""
        return matrix_to_pose(np.linalg.inv(self.matrix))

    def transform_point(self, point):
        """Exprime un point du repère enfant dans le repère parent.

        self = T_A<-B ; point exprimé en B -> retour exprimé en A.

        Args:
            point: array, liste ou tuple de forme (2,).

        Returns:
            np.ndarray: le point transformé, forme (2,).
        """
        point = np.asarray(point, dtype=float)
        if point.shape != (2,):
            raise ValueError("point must have shape (2,)")

        point_homogene = np.append(point, 1.0)
        resultat = self.matrix @ point_homogene
        return resultat[:2]


def normalize_angle(angle):
    return (angle + np.pi)%(2*np.pi) - np.pi

def matrix_to_pose(matrix):
    """Convertit une matrice homogène 3x3 en pose (x, y, theta)."""
    if not isinstance(matrix, np.ndarray) or matrix.shape != (3, 3):
        raise TypeError("matrix must be a 3x3 numpy array")
        
    x = matrix[0, 2]
    y = matrix[1, 2]
    theta = np.arctan2(matrix[1, 0], matrix[0, 0])
        
    return Pose2D(x, y, theta)





  