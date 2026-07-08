import numpy as np
from transforms2d import Pose2D, normalize_angle, matrix_to_pose


def test_theta_normalise():
    assert np.isclose(Pose2D(1.0, 2.0, 4*np.pi).theta, 0.0)

def test_accepte_les_entiers():
    assert isinstance(Pose2D(1, 2, 0).matrix, np.ndarray)

def test_rejette_les_chaines():
    import pytest
    with pytest.raises(TypeError):
        Pose2D("a", 2.0, 0.0)

def test_compose_identite():
    assert np.allclose(Pose2D(1.0, 2.0, np.pi).matrix,Pose2D(1.0, 2.0, np.pi).compose(Pose2D(0.0, 0.0, 0)).matrix) and  np.allclose(Pose2D(1.0, 2.0, np.pi).matrix, Pose2D(0.0, 0.0, 0).compose(Pose2D(1.0, 2.0, np.pi)).matrix) # compose avec l'identité : doit donner la même pose

def test_compose_cas_papier():
    resultat = Pose2D(1.0, 0.0, np.pi/2).compose(Pose2D(1.0, 0.0, 0.0))
    assert np.isclose(resultat.x, 1.0)
    assert np.isclose(resultat.y, 1.0)
    assert np.isclose(resultat.theta, np.pi/2)

def test_inverse():
    test_pose = Pose2D(1.0, 2.0, np.pi/4)
    assert np.allclose(test_pose.compose(test_pose.inverse()).matrix, np.eye(3))

def test_transform_point_cloture():
    robot = Pose2D(2.0, 0.0, np.pi/2)
    objet_monde = np.array([2.0, 3.0])
    objet_robot = robot.inverse().transform_point(objet_monde)
    assert np.allclose(objet_robot, [3.0, 0.0])